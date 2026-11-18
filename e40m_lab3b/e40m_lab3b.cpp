// Let's build a FLIP simulator! This is a very fun little project here, very
// approachable, but also a little bit challenging for someone like myself
// with no physics background.
//
// The original fluid-sim inspiration came from https://mitxela.com/projects/fluid-pendant
// I am following https://matthias-research.github.io/pages/tenMinutePhysics/18-flip.pdf
// as my primary resource here, as well as the associated video at https://youtu.be/XmzBREkK8kY
//
// I reccomend referencing the above resources as a primary source; the code
// will follow the slides almost exactly. I'm going to try to cite the slides
// for the steps; as it's an easy path to follow along with.
//
// Now, because we're running this on a teeny tiny card, actually implementing
// this is NOT trivial. Firstly, we have a grand total of 2KB of SRAM because
// Sam Altman took the rest of it for the datacenters. Or something like that.
//
// So this is how we will actually be implementing the code:
//
// Firstly, we need to use 16 bits for our floats here instead of the standard
// 32-bit float, because the ATmega328P has no floating-point hardware, and
// 4-byte floats would eat most of the 2KB on their own. So instead we use
// fixed point: a plain int16_t where we pretend the binary point sits 11 bits
// from the right.
//
// Now we're going to need these data structures for this implementation:
// -- AI --
// Feature: Two-board Communication
// Explanation: Was "const byte P = 40"; P now changes as water moves between boards.
//   byte P;                       // particles on this board right now, up to TOTAL = 40
// -- end AI --
//   fx xp[P], yp[P];              // where each particle is, in cells
//   fx up[P], vp[P];              // how fast it's going, in cells per step
//
//   const byte N = 10;            // 8x8 LEDs plus a ring of wall cells
//   fx u[N][N], v[N][N];          // the grid's velocities, on the cell edges
//   fx uCopy[N][N], vCopy[N][N];  // FLIP's copy of u and v
// -- AI --
// Feature: Two-board Communication
// Explanation: Was "weight totals, only while moving velocities onto the grid"; draw()
// now borrows r too, to save RAM.
//   fx r[N][N];                   // weight totals while moving velocities onto the grid; draw() borrows it
// -- end AI --
//   byte s[N][N];                 // 0 for wall cells, 1 for the rest
//   bool water[N][N];             // which cells have water in them
//
// -- AI --
// Feature: LED Brightness Control
// Explanation: Was "what the LEDs show"; ledOn now holds brightness, as on/off over 8
// ticks.
//   byte ledOn[8][8];             // which LEDs in each row are on, at each of 8 ticks
// -- end AI --
//
// Using this, we should be able to complete execution using only around 1.8 KB of
// memory at any given time.

// Our Fixed point format: Q4.11 (SIGNED 16-bit with 11 fractional bits)
//
// Ex 0b1011010001010101 can be thought of as:
// |       |   1  |   0110   | 10001010101 |
// | TYPE  | SIGN |   WHOLE  |  FRACTIONAL |
// | #bits |   1  |     4    |      11     |
// | Dec.  |      -10        |     1109    | <-- Divide fractional part by 2048
// Value =  -10 + 1109/2048 = -9.45849609375
//
// Another way to think about it is just as the decimal value of the binary bits divided by 2048
// Ex 0b1011010001010101 = −19371 / 2048. = -9.45849609375

// -- AI --
// Feature: Two-board Communication
// Explanation: Two boards share the water: a particle leaving one board's rim (row A)
// drops in through the other's, over the serial link further down. These are numbers
// both boards agree on, and a Drop is one particle on its way across.
const byte TOTAL = 40;    // all the water in both boards; one board can hold all of it
const byte MAX_DROPS = 4; // most particles that can leave through the rim in one step
struct Drop               // DROP payload bytes exactly as sent (sender's frame)
{
  byte pos, vx, vy;
};
// -- end AI --

typedef int16_t fx;
const fx ONE = 2048; // 1.0
// -- AI --
// Feature: Optimization
// Explanation: Originally (long)a * b / ONE, but dividing proved too slow in practice.
// Shifting right by 11 bits does the same job, but even that is an 11-step loop for a
// 32-bit number on this chip, so this builds the answer out of two cheap 16-bit shifts.
// It's also always inlined, since it's called hundreds of times every step.
inline __attribute__((always_inline)) fx fxmul(fx a, fx b)
{
  long p = (long)a * b;
  // Same as p >> 11: p's top half moved up 5 fills bits 5-15 of the answer, and its
  // bottom half moved down 11 fills bits 0-4. (p >> 16 costs nothing; it's just p's
  // top two bytes.)
  return (int)(p >> 16) << 5 | (unsigned)p >> 11;
}
// -- end AI --
fx fxdiv(fx a, fx b) { return (long)a * ONE / b; }

const byte N = 10; // 10 items in grid (8x8 plus walls)
// -- AI --
// Feature: Two-board Communication
// Explanation: Originally a constant P = 40, with the arrays sized to it. Now P is how
// many particles this board holds right now, which changes as water crosses between
// the boards, so the arrays have room for the most it could ever hold: all of it.
byte P; // num particles on this board; the rest are on the other one
fx xp[TOTAL], yp[TOTAL];
fx up[TOTAL], vp[TOTAL];
// -- end AI --
const fx radius = 0.45 * ONE; // a bit under half a cell, so each particle is about one LED across
fx u[N][N], v[N][N]; // The grid's velocities sit on the cell edges (slide 4).
fx uCopy[N][N], vCopy[N][N]; // the copy from slide 11
// -- AI --
// Feature: Two-board Communication
// Explanation: Originally a local array inside particlesToGrid(). It's global now so
// draw() can borrow the same 200 bytes: once Serial took its share of the 2 KB of RAM,
// the stack no longer had room for draw() to have an array of its own.
fx r[N][N];       // weight totals in particlesToGrid(); draw() borrows it
// -- end AI --
byte s[N][N];     // 0 for walls, 1 for everything else (slide 21)
bool water[N][N]; // the water cells (slide 8). Anything else that isn't a wall is air.
fx gx, gy; // gravity, from the accelerometer
// -- AI --
// Feature: Two-board Communication
// Explanation: Sent to the other board, which only opens its rim (to pour into this
// one) while this board's rim is uphill. See readTilt() and rimOpen().
int8_t gn; // gravity toward the rim (row A) in 1/64 g, for the other board
// -- end AI --
// -- AI --
// Feature: Auto-Leveling
// Explanation: What the accelerometer reads when the board is level, measured at
// power-on instead of hardcoded.
long xFlat, yFlat; // 32 accelerometer readings added up while lying flat (see findLevel())
// -- end AI --

const byte ANODE_PINS[8] = {13, 12, 11, 10, 9, 8, 7, 6};   // LED rows, top to bottom
const byte CATHODE_PINS[8] = {A3, A2, A1, A0, 5, 4, 3, 2}; // LED columns, left to right
volatile byte ledOn[8][8];                                 // volatile because the display interrupt reads it

// Slide 8: Cell Type Determination
void findWaterCells()
{
  for (byte i = 0; i < N; i++)
    for (byte j = 0; j < N; j++)
      water[i][j] = false;
  for (byte i = 0; i < P; i++)
    water[xp[i] / ONE][yp[i] / ONE] = true; // particle? = water
}

// Slide 12: Simulate Particles
void simulateParticles()
{
  for (byte i = 0; i < P; i++)
  {
    up[i] += gx;
    vp[i] += gy;

    // no faster than one cell per step
    up[i] = constrain(up[i], -ONE, ONE);
    vp[i] = constrain(vp[i], -ONE, ONE);

    xp[i] += up[i];
    yp[i] += vp[i];
  }
}

// -- AI --
// Feature: Two-board Communication
// Explanation: Originally took and returned nothing. While the rim is open, a particle
// going out past the top wall now leaves through it, into out[] (if there's room),
// instead of being pushed back in. Returns how many left.
byte pushParticlesOutOfWalls(bool openRim, Drop out[MAX_DROPS])
// -- end AI --
{
  fx lo = ONE + radius; // the walls' inside edges are at 1 and 9
  fx hi = (N - 1) * ONE - radius;
// -- AI --
// Feature: Two-board Communication
// Explanation: n counts the particles that leave. The loop used to run forwards; it
// runs backwards now, because a leaving particle's slot is filled by the last
// particle, which has then already been checked.
  byte n = 0;
  for (byte i = P; i-- > 0;) // i = P - 1 down to 0
// -- end AI --
  {
    if (xp[i] < lo)
    {
      xp[i] = lo;
      up[i] = 0;
    }
    if (xp[i] > hi)
    {
      xp[i] = hi;
      up[i] = 0;
    }
// -- AI --
// Feature: Two-board Communication
// Explanation: While the rim is open, a particle past the top wall that's still moving
// out (not one that pushParticlesApart() just pushed there) leaves through the rim:
// it's packed into a Drop for the other board, and removed from this one.
    if (yp[i] > hi && vp[i] > 0 && openRim && n < MAX_DROPS)
    {
      // The DROP packet's bytes, each 0 to 127: where it is along the rim, in 1/16 cell
      // from the left wall's inside edge (>> 7), and its velocity in 1/32 cell per
      // step (>> 6), plus 64 so it isn't negative
      out[n].pos = (xp[i] - ONE) >> 7;
      out[n].vx = (up[i] >> 6) + 64;
      out[n].vy = (vp[i] >> 6) + 64;
      n++;

      // the last particle takes its slot
      P--;
      xp[i] = xp[P];
      yp[i] = yp[P];
      up[i] = up[P];
      vp[i] = vp[P];
      continue;
    }
// -- end AI --
    if (yp[i] < lo)
    {
      yp[i] = lo;
      vp[i] = 0;
    }
    if (yp[i] > hi)
    {
      yp[i] = hi;
      vp[i] = 0;
    }
  }
// -- AI --
// Feature: Two-board Communication
// Explanation: How many particles left through the rim.
  return n;
// -- end AI --
}

// Slide 15: From Grid to Particles
// FLIP (slide 11): add how much the grid changed, not the grid itself
void gridToParticles()
{
  for (byte p = 0; p < P; p++)
  {
    // u is half a cell lower (slide 16)
    fx x = xp[p];
    fx y = yp[p] - ONE / 2;
// -- AI --
// Feature: Optimization
// Explanation: Originally a division by ONE, but proved too slow in practice; a bit
// shift is quick. Likewise, dx and dy were a remainder (% ONE), and are now a bit mask
// that keeps the 11 fraction bits. Same results, since x and y are never negative.
    byte i = x >> 11; // slide 14
    byte j = y >> 11;
    fx dx = x & (ONE - 1);
    fx dy = y & (ONE - 1);
// -- end AI --
    fx w1 = fxmul(ONE - dx, ONE - dy);
    fx w2 = fxmul(dx, ONE - dy);
    fx w3 = fxmul(dx, dy);
// -- AI --
// Feature: Optimization
// Explanation: Originally fxmul(ONE - dx, dy). What's left over is practically the
// same weight, but makes the 4 add up to exactly ONE, which the shortcut below needs.
    fx w4 = ONE - w1 - w2 - w3;
// -- end AI --

    // no water on either side = undefined (slide 7)
    if (!water[i - 1][j] && !water[i][j])
      w1 = 0;
    if (!water[i][j] && !water[i + 1][j])
      w2 = 0;
    if (!water[i][j + 1] && !water[i + 1][j + 1])
      w3 = 0;
    if (!water[i - 1][j + 1] && !water[i][j + 1])
      w4 = 0;

    fx q1 = u[i][j] - uCopy[i][j];
    fx q2 = u[i + 1][j] - uCopy[i + 1][j];
    fx q3 = u[i + 1][j + 1] - uCopy[i + 1][j + 1];
    fx q4 = u[i][j + 1] - uCopy[i][j + 1];
    fx total = w1 + w2 + w3 + w4;
// -- AI --
// Feature: Optimization
// Explanation: Originally always divided the weighted sum by the total weight. When no
// weight was zeroed above, the total is exactly ONE and dividing by it changes
// nothing, so the slow divide is skipped.
    fx sum = fxmul(w1, q1) + fxmul(w2, q2) + fxmul(w3, q3) + fxmul(w4, q4);
    if (total == ONE)
      up[p] += sum;
    else if (total > 0)
      up[p] += fxdiv(sum, total);
// -- end AI --

    // v is half a cell left (slide 16)
    x = xp[p] - ONE / 2;
    y = yp[p];
// -- AI --
// Feature: Optimization
// Explanation: As above: a bit shift and mask instead of dividing by ONE.
    i = x >> 11;
    j = y >> 11;
    dx = x & (ONE - 1);
    dy = y & (ONE - 1);
// -- end AI --
    w1 = fxmul(ONE - dx, ONE - dy);
    w2 = fxmul(dx, ONE - dy);
    w3 = fxmul(dx, dy);
// -- AI --
// Feature: Optimization
// Explanation: As above, so the 4 weights add up to exactly ONE.
    w4 = ONE - w1 - w2 - w3;
// -- end AI --

    // no water on either side = undefined (slide 7)
    if (!water[i][j - 1] && !water[i][j])
      w1 = 0;
    if (!water[i + 1][j - 1] && !water[i + 1][j])
      w2 = 0;
    if (!water[i + 1][j] && !water[i + 1][j + 1])
      w3 = 0;
    if (!water[i][j] && !water[i][j + 1])
      w4 = 0;

    q1 = v[i][j] - vCopy[i][j];
    q2 = v[i + 1][j] - vCopy[i + 1][j];
    q3 = v[i + 1][j + 1] - vCopy[i + 1][j + 1];
    q4 = v[i][j + 1] - vCopy[i][j + 1];
    total = w1 + w2 + w3 + w4;
// -- AI --
// Feature: Optimization
// Explanation: As above, skipping the divide when the total weight is exactly ONE.
    sum = fxmul(w1, q1) + fxmul(w2, q2) + fxmul(w3, q3) + fxmul(w4, q4);
    if (total == ONE)
      vp[p] += sum;
    else if (total > 0)
      vp[p] += fxdiv(sum, total);
// -- end AI --
  }
}

// Slide 17: From Particles to Grid
void particlesToGrid()
{
  // clear q and r of all cells
  for (byte i = 0; i < N; i++)
    for (byte j = 0; j < N; j++)
    {
      u[i][j] = 0;
      v[i][j] = 0;
    }

  // u, half a cell lower (slide 16)
  for (byte i = 0; i < N; i++)
    for (byte j = 0; j < N; j++)
      r[i][j] = 0;
  for (byte p = 0; p < P; p++)
  {
    fx x = xp[p];
    fx y = yp[p] - ONE / 2;
// -- AI --
// Feature: Optimization
// Explanation: As in gridToParticles(): a bit shift and mask instead of dividing by ONE.
    byte i = x >> 11;
    byte j = y >> 11;
    fx dx = x & (ONE - 1);
    fx dy = y & (ONE - 1);
// -- end AI --
    fx w1 = fxmul(ONE - dx, ONE - dy);
    fx w2 = fxmul(dx, ONE - dy);
    fx w3 = fxmul(dx, dy);
    fx w4 = fxmul(ONE - dx, dy);

    u[i][j] += fxmul(w1, up[p]);
    u[i + 1][j] += fxmul(w2, up[p]);
    u[i + 1][j + 1] += fxmul(w3, up[p]);
    u[i][j + 1] += fxmul(w4, up[p]);
    r[i][j] += w1;
    r[i + 1][j] += w2;
    r[i + 1][j + 1] += w3;
    r[i][j + 1] += w4;
  }
  for (byte i = 0; i < N; i++)
    for (byte j = 0; j < N; j++)
      if (r[i][j] > 0)
        u[i][j] = fxdiv(u[i][j], r[i][j]);

  // v, half a cell left (slide 16)
  for (byte i = 0; i < N; i++)
    for (byte j = 0; j < N; j++)
      r[i][j] = 0;
  for (byte p = 0; p < P; p++)
  {
    fx x = xp[p] - ONE / 2;
    fx y = yp[p];
// -- AI --
// Feature: Optimization
// Explanation: As in gridToParticles(): a bit shift and mask instead of dividing by ONE.
    byte i = x >> 11;
    byte j = y >> 11;
    fx dx = x & (ONE - 1);
    fx dy = y & (ONE - 1);
// -- end AI --
    fx w1 = fxmul(ONE - dx, ONE - dy);
    fx w2 = fxmul(dx, ONE - dy);
    fx w3 = fxmul(dx, dy);
    fx w4 = fxmul(ONE - dx, dy);

    v[i][j] += fxmul(w1, vp[p]);
    v[i + 1][j] += fxmul(w2, vp[p]);
    v[i + 1][j + 1] += fxmul(w3, vp[p]);
    v[i][j + 1] += fxmul(w4, vp[p]);
    r[i][j] += w1;
    r[i + 1][j] += w2;
    r[i + 1][j + 1] += w3;
    r[i][j + 1] += w4;
  }
  for (byte i = 0; i < N; i++)
    for (byte j = 0; j < N; j++)
      if (r[i][j] > 0)
        v[i][j] = fxdiv(v[i][j], r[i][j]);

  // nothing flows through a wall
  for (byte i = 1; i < N; i++)
    for (byte j = 1; j < N; j++)
    {
      if (s[i - 1][j] == 0 || s[i][j] == 0)
        u[i][j] = 0;
      if (s[i][j - 1] == 0 || s[i][j] == 0)
        v[i][j] = 0;
    }

  // make a copy (slide 11)
  for (byte i = 0; i < N; i++)
    for (byte j = 0; j < N; j++)
    {
      uCopy[i][j] = u[i][j];
      vCopy[i][j] = v[i][j];
    }
}

// Slides 19 to 23: Making the velocity field incompressible
void solveIncompressibility()
{
// -- AI --
// Feature: Optimization
// Explanation: Originally d was divided by sSum for every water cell, 10 times a step.
// sSum can only be 0 to 4, so o / sSum has just 5 possible values: they're worked out
// up front, and looked up below instead.
  const fx o = 1.9 * ONE; // overrelaxation (slide 23)
  const fx oOverS[5] = {0, o, o / 2, o / 3, o / 4};
// -- end AI --
  for (byte n = 0; n < 10; n++)
  {
    for (byte i = 1; i < N - 1; i++)
    {
      for (byte j = 1; j < N - 1; j++)
      {
        if (!water[i][j])
          continue; // don't process air cells (slide 7)

        byte sSum = s[i + 1][j] + s[i - 1][j] + s[i][j + 1] + s[i][j - 1];
// -- AI --
// Feature: Optimization
// Explanation: Originally fxmul(o, ...) here, followed by d = d / sSum; multiplying by
// o / sSum from the table does both at once, with no divide.
        fx d = fxmul(oOverS[sSum],u[i + 1][j] - u[i][j] + v[i][j + 1] - v[i][j]);
// -- end AI --
        u[i][j] += d * s[i - 1][j];
        u[i + 1][j] -= d * s[i + 1][j];
        v[i][j] += d * s[i][j - 1];
        v[i][j + 1] -= d * s[i][j + 1];
      }
    }
  }
}

// -- AI --
// Feature: Optimization
// Explanation: Square root of n, rounded down, using only integer math: it works out
// the root one bit at a time, from the top. pushParticlesApart() uses it instead of
// sqrt(), which is slow because this chip has no floating-point hardware.
fx isqrt(long n)
{
  long root = 0;
  for (long bit = 1L << 22; bit > 0; bit >>= 2) // n is under 2^24 here
  {
    if (n >= root + bit)
    {
      n -= root + bit;
      root = root / 2 + bit;
    }
    else
      root /= 2;
  }
  return root;
}
// -- end AI --

// Slide 26: Push Particles Apart, Simply
void pushParticlesApart()
{
  fx minDist = 2 * radius;
  for (byte i = 0; i < P; i++)
  {
    for (byte j = i + 1; j < P; j++)
    {
      fx dx = xp[j] - xp[i];
      fx dy = yp[j] - yp[i];
      if (abs(dx) >= minDist || abs(dy) >= minDist)
        continue; // not close
// -- AI --
// Feature: Optimization
// Explanation: Originally took the square root first, then compared the distance to
// minDist. Comparing their squares gives the same answer, so pairs that aren't
// touching skip the square root, and the rest use the faster isqrt() above.
      long d2 = (long)dx * dx + (long)dy * dy;
      if (d2 >= (long)minDist * minDist)
        continue; // not touching
      fx d = isqrt(d2);
// -- end AI --

      // right on top of each other; pick a direction
      if (d == 0)
      {
        dx = 1;
        d = 1;
      }

      // each moves half the overlap
// -- AI --
// Feature: Optimization
// Explanation: Originally divided by d (and then by 2) separately for each direction.
// Working out the push per unit of distance once, with the half built in (<< 10
// instead of << 11), takes one slow division per pair instead of two.
      long push = ((long)(minDist - d) << 10) / d; // half the overlap, per unit of distance
      fx moveX = dx * push >> 11;
      fx moveY = dy * push >> 11;
// -- end AI --
      xp[i] -= moveX;
      yp[i] -= moveY;
      xp[j] += moveX;
      yp[j] += moveY;
    }
  }
}

// Slide 11, plus pushing apart
// -- AI --
// Feature: Two-board Communication
// Explanation: Originally step(), with no arguments. Now it's told whether the rim is
// open, puts the particles that leave through it in out[], and returns how many did.
byte simStep(bool openRim, Drop out[MAX_DROPS])
{
  for (byte i = 1; i < N - 1; i++)
    s[i][N - 1] = openRim; // an open rim's cells are air, so the pressure solve can push water out
// -- end AI --
  simulateParticles();
  pushParticlesApart();
// -- AI --
// Feature: Two-board Communication
// Explanation: Passes on whether the rim is open, and where to put particles that leave.
  byte n = pushParticlesOutOfWalls(openRim, out); // pushing apart can push some into a wall
// -- end AI --
  findWaterCells();
  particlesToGrid();
  solveIncompressibility();
  gridToParticles();
// -- AI --
// Feature: Two-board Communication
// Explanation: How many particles left through the rim.
  return n;
// -- end AI --
}

// -- AI --
// Feature: Two-board Communication
// Explanation: Starts over with `count` particles, as a block in the bottom left, 6
// wide. This is the original particle setup from setup(), moved here so a RESET from
// the other board can rerun it, which is also why it now zeroes the velocities.
void simReset(byte count)
{
  P = min(count, TOTAL);
  for (byte i = 0; i < P; i++)
  {
    xp[i] = ONE + radius + (i % 6) * 2 * radius;
    yp[i] = ONE + radius + (i / 6) * 2 * radius;
    up[i] = 0;
    vp[i] = 0;
  }
}
// -- end AI --

// -- AI --
// Feature: Two-board Communication
// Explanation: Brings in a particle from the other board. The two rims face each
// other, so it turns 180 degrees on the way: where it crossed is mirrored, and its
// velocity is flipped. It comes in just inside the rim.
void simImport(Drop d)
{
  if (P == TOTAL)
    return; // full
  // pos is in 1/16 cell from the left wall's inside edge, and the rim is 8 cells =
  // 128 16ths long, so 128 - pos is its mirror image. 1/16 cell is 128 in fx.
  xp[P] = ONE + (128 - d.pos) * 128;
  yp[P] = (N - 1) * ONE - radius; // just inside the rim
  // the velocities come in 1/32 cell per step, plus 64; 1/32 cell is 64 in fx
  up[P] = -(d.vx - 64) * 64;
  vp[P] = -(d.vy - 64) * 64;
  P++;
}
// -- end AI --

// -- AI --
// Feature: Two-board Communication
// Explanation: The link to the other board, over Serial at 38400 baud (pins 0 and 1,
// crossed, grounds joined). Every packet is a header byte with the top bit set, then
// payload bytes with it clear, then a check byte: the sum of everything before it,
// & 0x7F. Since only headers have the top bit set, a board can always find where the
// next packet starts, and the check byte lets it throw out one that got garbled.
//   RESET  81 52 chk          both boards start over with half the water each
//   STATE  82 gn count chk    my gravity toward my rim, + 64, and how many I hold
//   DROP   83 pos vx vy chk   a particle leaving my rim, in my frame
byte packet[5];          // the packet coming in, header first
byte got = 0;            // how many of its bytes are in
byte want = 0;           // how long it is; 0 while skipping to the next header
int8_t peerGn = 0;       // the other board's gravity toward its rim, from its last STATE
unsigned long lastGood;  // when the last good packet came in
unsigned badPackets = 0; // check byte didn't match: the first thing to look at when debugging

void linkBegin()
{
  Serial.begin(38400);
  pinMode(0, INPUT_PULLUP); // an unplugged cable then reads idle instead of noise
  lastGood = millis() - 250; // down until the other board says something
}

// Reads every byte that has come in, and acts on each complete packet that checks out
void linkPoll()
{
  while (Serial.available())
  {
    byte b = Serial.read();
    if (b & 0x80) // a header always starts a new packet, dropping any half-read one
    {
      got = 0;
      want = b == 0x81 ? 3 : b == 0x82 ? 4 : b == 0x83 ? 5 : 0; // its length; unknown type: skip it
    }
    if (want == 0)
      continue; // between packets, so debug text and unknown packets end up here
    packet[got++] = b;
    if (got < want)
      continue;
    want = 0;

    byte sum = 0;
    for (byte i = 0; i < got - 1; i++)
      sum += packet[i];
    if ((sum & 0x7F) != packet[got - 1])
    {
      badPackets++;
      continue;
    }

    lastGood = millis();
    if (packet[0] == 0x81) // RESET
      simReset(TOTAL / 2);
    else if (packet[0] == 0x82) // STATE
      peerGn = packet[1] - 64; // its count, packet[2], is unused for now
    else // DROP
      simImport({packet[1], packet[2], packet[3]});
  }
}

// Sends a header and payload, then their check byte
void linkSend(const byte *bytes, byte n)
{
  byte sum = 0;
  for (byte i = 0; i < n; i++)
  {
    Serial.write(bytes[i]);
    sum += bytes[i];
  }
  Serial.write(sum & 0x7F);
}

void linkSendState(int8_t gn, byte count)
{
  byte b[] = {0x82, (byte)(gn + 64), count};
  linkSend(b, 3);
}

void linkSendDrop(Drop d)
{
  byte b[] = {0x83, d.pos, d.vx, d.vy};
  linkSend(b, 4);
}

void linkSendReset()
{
  byte b[] = {0x81, 'R'};
  linkSend(b, 2);
}

// Up while a good packet has come in within the last 250 ms
bool linkUp() { return millis() - lastGood < 250; }

// Open only while the other board's rim is uphill by more than about 0.1 g, so
// water falls into it and can't climb back out of it
bool rimOpen() { return linkUp() && peerGn < -6; }
// -- end AI --

// ADXL335 on A4 (x) and A5 (y): about 335 / 330 when level, 67 counts per g.
// These values change on battery; it's annoying. These values work on USB for now though.
void readTilt()
{
  fx g = 0.2 * ONE; // full tilt, tune to taste
// -- AI --
// Feature: Auto-Leveling
// Explanation: Originally subtracted the 335 / 330 above, which are only right on USB.
// Now it subtracts what findLevel() measured at power-on. Those are 32 readings added
// up, so the reading is multiplied by 32 to match, and the 32 divided back out.
  gx = (32L * analogRead(A4) - xFlat) * g / (67 * 32);
  gy = (32L * analogRead(A5) - yFlat) * g / (67 * 32);
// -- end AI --
// -- AI --
// Feature: Two-board Communication
// Explanation: gy again, but in 1/64 g (gy / g is in g), for the other board. It's
// clamped so that gn + 64 fits in a packet byte.
  gn = constrain(gy * 64L / g, -64, 63);
// -- end AI --
}

// -- AI --
// Feature: Auto-Leveling
// Explanation: The accelerometer isn't mounted perfectly flat, so even lying flat it
// feels a little tilt and the water slides. So whatever it reads at power-on counts as
// level: turn it on lying flat! Adding up 32 readings keeps the fraction of a count
// that a single reading would lose.
void findLevel()
{
  delay(500); // let it stop wobbling after being set down
  xFlat = 0;
  yFlat = 0;
  for (byte n = 0; n < 32; n++)
  {
    xFlat += analogRead(A4);
    yFlat += analogRead(A5);
    delay(10);
  }
}
// -- end AI --

// LED row 0 is the top (y = 8), column 0 is the left (x = 1)
void draw()
{
// -- AI --
// Feature: Two-board Communication
// Explanation: draw() needs a 200-byte scratch array, but once Serial took its share
// of the RAM, the stack had no room left for one. So it borrows r, which only matters
// inside particlesToGrid(), and clears it.
  fx (*amount)[N] = r;
  memset(r, 0, sizeof r);
// -- end AI --
// -- AI --
// Feature: LED Brightness Control
// Explanation: Originally an LED was simply on if its cell had water in it. Now each
// LED's brightness shows how much water is near it: every particle adds to the 4 LEDs
// around it, more to the closer ones (the weights from slide 14), so the water fades
// from one LED into the next as it moves.
  for (byte p = 0; p < P; p++)
  {
    fx x = xp[p] - ONE / 2; // LED centers are half a cell in
    fx y = yp[p] - ONE / 2;
    byte i = x >> 11;
    byte j = y >> 11;
    fx dx = x & (ONE - 1);
    fx dy = y & (ONE - 1);
    amount[i][j] += fxmul(ONE - dx, ONE - dy);
    amount[i + 1][j] += fxmul(dx, ONE - dy);
    amount[i + 1][j + 1] += fxmul(dx, dy);
    amount[i][j + 1] += fxmul(ONE - dx, dy);
  }
// -- end AI --

// -- AI --
// Feature: LED Brightness Control, Two-board Communication
// Explanation: Brightness is 8 levels per particle (amount >> 8, up to 8), and an LED
// with brightness b is on for the first b of its row's 8 ticks. Which LEDs are on at
// each tick is worked out here, 25 times a second, so the display interrupt (8000
// times a second) only has to copy it out. For the link: while it's down, row A's two
// corner LEDs blink about twice a second, whatever water is there.
  bool linkDown = !linkUp();
  byte corners = millis() & 256 ? 0b10000001 : 0; // both corners, on or off for 256 ms at a time
  for (byte row = 0; row < 8; row++)
  {
    byte bright[8];
    for (byte col = 0; col < 8; col++)
      bright[col] = min(amount[col + 1][8 - row] >> 8, 8);
    for (byte tick = 0; tick < 8; tick++)
    {
      byte on = 0;
      for (byte col = 0; col < 8; col++)
        on = on << 1 | (bright[col] > tick); // column 0 ends up in the top bit
      if (row == 0 && linkDown)
        on = (on & 0b01111110) | corners; // the link-down blink
      ledOn[row][tick] = on;
    }
  }
// -- end AI --
}

// -- AI --
// Feature: LED Brightness Control
// Explanation: Originally display() from lab 3a, one row per call, about once a
// millisecond. For brightness, each row's turn is now split into 8 ticks, and
// ledOn[row][tick] (worked out by draw()) says which of its LEDs are on during each
// one, so this runs 8000 times a second. That's too often for digitalWrite(), so it
// sets the pins through their port registers instead.
ISR(TIMER2_COMPA_vect)
{
  // Rows A-F are PORTB bits 5-0 and G-H are PORTD bits 7-6; columns 1-4 are PORTC
  // bits 3-0 and 5-8 are PORTD bits 5-2. All of them are LOW = on.
  static byte row = 0, tick = 0;
  if (tick == 0)
  {
    PORTB |= 0b00111111; // last row off
    PORTD |= 0b11000000;
    row = (row + 1) % 8;
  }

  // All 8 columns HIGH (off), then the ones that are on this tick pulled LOW, leaving
  // the ports' other pins as they were
  byte on = ledOn[row][tick];
  PORTC = (PORTC | 0b00001111) & ~(on >> 4);         // columns 1-4: on's top 4 bits, into bits 3-0
  PORTD = (PORTD | 0b00111100) & ~((on & 0x0F) << 2); // columns 5-8: on's low 4 bits, into bits 5-2

  if (tick == 0) // next row on
  {
    if (row < 6)
      PORTB &= ~(0b00100000 >> row);
    else
      PORTD &= ~(0b10000000 >> (row - 6));
  }
  tick = (tick + 1) % 8;
}
// -- end AI --

void setup()
{
  for (byte i = 0; i < 8; i++)
  {
    pinMode(ANODE_PINS[i], OUTPUT);
    pinMode(CATHODE_PINS[i], OUTPUT);
    digitalWrite(ANODE_PINS[i], HIGH);
    digitalWrite(CATHODE_PINS[i], HIGH);
  }

  // border cells are walls (slide 22)
  for (byte i = 0; i < N; i++)
    for (byte j = 0; j < N; j++)
      s[i][j] = (i == 0 || j == 0 || i == N - 1 || j == N - 1) ? 0 : 1;

// -- AI --
// Feature: Two-board Communication
// Explanation: The particle setup that was here is simReset() now, so a RESET from
// the other board can rerun it. Each board starts with half the water.
  simReset(TOTAL / 2); // half the water each
// -- end AI --
// -- AI --
// Feature: Auto-Leveling
// Explanation: Fills in ledOn before the display starts, so the starting block shows
// during the 0.8 s findLevel() takes, instead of a blank screen.
  draw();
// -- end AI --

// -- AI --
// Feature: LED Brightness Control
// Explanation: Originally the display piggybacked on Timer0 (the timer behind
// millis()), about once a millisecond. With 8 ticks per row it needs 8 times that, so
// it gets Timer2 to itself: 8000 interrupts a second, still about 1 ms per row.
  TCCR2A = 1 << WGM21;  // count up to OCR2A, then start over
  TCCR2B = 1 << CS22;   // 16 MHz / 64 = 250 kHz
  OCR2A = 30;           // 31 counts = 124 us
  TIMSK2 = 1 << OCIE2A; // interrupt each time it gets to OCR2A
// -- end AI --

// -- AI --
// Feature: Auto-Leveling
// Explanation: Measures what the accelerometer reads when level (see findLevel()).
  findLevel();
// -- end AI --

// -- AI --
// Feature: Two-board Communication
// Explanation: Starts the link. Since every boot sends RESET, whichever board boots last
// syncs both, so pressing reset on either board restarts the pair.
  linkBegin();
  linkSendReset(); // the other board starts over with half the water too
// -- end AI --
}

void loop()
{
// -- AI --
// Feature: Two-board Communication
// Explanation: Handles whatever the other board has sent. It runs on every pass, not
// just when a step is due, so Serial's 64-byte receive buffer keeps getting emptied.
  linkPoll();
// -- end AI --

  // at most 25 steps a second
  static unsigned long lastStep = 0;
  if (millis() - lastStep < 40)
    return;
  lastStep = millis();

  readTilt();
// -- AI --
// Feature: Two-board Communication
// Explanation: Originally just step(). Now the step is told whether the rim is open,
// the particles that leave through it go to the other board as DROP packets, and a
// STATE packet tells it this board's gravity toward its rim and how many it holds.
  Drop out[MAX_DROPS];
  byte n = simStep(rimOpen(), out);
  for (byte i = 0; i < n; i++)
    linkSendDrop(out[i]);
  linkSendState(gn, P);
// -- end AI --
  draw();
}
