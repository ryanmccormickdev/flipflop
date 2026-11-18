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
//   const byte P = 40;            // particles
//   fx xp[P], yp[P];              // where each particle is, in cells
//   fx up[P], vp[P];              // how fast it's going, in cells per step
//
//   const byte N = 10;            // 8x8 LEDs plus a ring of wall cells
//   fx u[N][N], v[N][N];          // the grid's velocities, on the cell edges
//   fx uCopy[N][N], vCopy[N][N];  // FLIP's copy of u and v
//   fx r[N][N];                   // weight totals, only while moving velocities onto the grid
//   byte s[N][N];                 // 0 for wall cells, 1 for the rest
//   bool water[N][N];             // which cells have water in them
//
//   byte ledOn[8][8];             // what the LEDs show
//
// Using this, we should be able to complete execution using only around 1.6 KB of
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

typedef int16_t fx;
const fx ONE = 2048; // 1.0
fx fxmul(fx a, fx b) { return (long)a * b / ONE; }
fx fxdiv(fx a, fx b) { return (long)a * ONE / b; }

const byte N = 10; // 10 items in grid (8x8 plus walls)
const byte P = 40; // num particles
fx xp[P], yp[P];
fx up[P], vp[P];
const fx radius = 0.45 * ONE; // a bit under half a cell, so each particle is about one LED across
fx u[N][N], v[N][N]; // The grid's velocities sit on the cell edges (slide 4).
fx uCopy[N][N], vCopy[N][N]; // the copy from slide 11
byte s[N][N];     // 0 for walls, 1 for everything else (slide 21)
bool water[N][N]; // the water cells (slide 8). Anything else that isn't a wall is air.
fx gx, gy; // gravity, from the accelerometer

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

void pushParticlesOutOfWalls()
{
  fx lo = ONE + radius; // the walls' inside edges are at 1 and 9
  fx hi = (N - 1) * ONE - radius;
  for (byte i = 0; i < P; i++)
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
    byte i = x / ONE; // slide 14
    byte j = y / ONE;
    fx dx = x % ONE;
    fx dy = y % ONE;
    fx w1 = fxmul(ONE - dx, ONE - dy);
    fx w2 = fxmul(dx, ONE - dy);
    fx w3 = fxmul(dx, dy);
    fx w4 = fxmul(ONE - dx, dy);

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
    if (total > 0)
      up[p] += fxdiv(fxmul(w1, q1) + fxmul(w2, q2) + fxmul(w3, q3) + fxmul(w4, q4), total);

    // v is half a cell left (slide 16)
    x = xp[p] - ONE / 2;
    y = yp[p];
    i = x / ONE;
    j = y / ONE;
    dx = x % ONE;
    dy = y % ONE;
    w1 = fxmul(ONE - dx, ONE - dy);
    w2 = fxmul(dx, ONE - dy);
    w3 = fxmul(dx, dy);
    w4 = fxmul(ONE - dx, dy);

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
    if (total > 0)
      vp[p] += fxdiv(fxmul(w1, q1) + fxmul(w2, q2) + fxmul(w3, q3) + fxmul(w4, q4), total);
  }
}

// Slide 17: From Particles to Grid
void particlesToGrid()
{
  fx r[N][N];

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
    byte i = x / ONE;
    byte j = y / ONE;
    fx dx = x % ONE;
    fx dy = y % ONE;
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
    byte i = x / ONE;
    byte j = y / ONE;
    fx dx = x % ONE;
    fx dy = y % ONE;
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
  fx o = 1.9 * ONE; // overrelaxation (slide 23)
  for (byte n = 0; n < 10; n++)
  {
    for (byte i = 1; i < N - 1; i++)
    {
      for (byte j = 1; j < N - 1; j++)
      {
        if (!water[i][j])
          continue; // don't process air cells (slide 7)

        fx d = fxmul(o, u[i + 1][j] - u[i][j] + v[i][j + 1] - v[i][j]);
        byte sSum = s[i + 1][j] + s[i - 1][j] + s[i][j + 1] + s[i][j - 1];
        d = d / sSum;
        u[i][j] += d * s[i - 1][j];
        u[i + 1][j] -= d * s[i + 1][j];
        v[i][j] += d * s[i][j - 1];
        v[i][j + 1] -= d * s[i][j + 1];
      }
    }
  }
}

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
      fx d = sqrt((long)dx * dx + (long)dy * dy);
      if (d >= minDist)
        continue;

      // right on top of each other; pick a direction
      if (d == 0)
      {
        dx = 1;
        d = 1;
      }

      // each moves half the overlap
      fx moveX = (long)dx * (minDist - d) / d / 2;
      fx moveY = (long)dy * (minDist - d) / d / 2;
      xp[i] -= moveX;
      yp[i] -= moveY;
      xp[j] += moveX;
      yp[j] += moveY;
    }
  }
}

// Slide 11, plus pushing apart
void step()
{
  simulateParticles();
  pushParticlesApart();
  pushParticlesOutOfWalls(); // pushing apart can push some into a wall
  findWaterCells();
  particlesToGrid();
  solveIncompressibility();
  gridToParticles();
}

// ADXL335 on A4 (x) and A5 (y): about 335 / 330 when level, 67 counts per g.
// These values change on battery; it's annoying. These values work on USB for now though.
void readTilt()
{
  fx g = 0.2 * ONE; // full tilt, tune to taste
  gx = (long)(analogRead(A4) - 335) * g / 67;
  gy = (long)(analogRead(A5) - 330) * g / 67;
}

// LED row 0 is the top (y = 8), column 0 is the left (x = 1)
void draw()
{
  for (byte row = 0; row < 8; row++)
    for (byte col = 0; col < 8; col++)
      ledOn[row][col] = water[col + 1][8 - row];
}

// display() from lab 3a, but one row per call, about once a millisecond
ISR(TIMER0_COMPA_vect)
{
  static byte row = 0;
  digitalWrite(ANODE_PINS[row], HIGH); // last row off
  row = (row + 1) % 8;
  for (byte col = 0; col < 8; col++)
    digitalWrite(CATHODE_PINS[col], ledOn[row][col] ? LOW : HIGH);
  digitalWrite(ANODE_PINS[row], LOW); // next row on
}

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

  // start as a block in the bottom left, 6 wide
  for (byte i = 0; i < P; i++)
  {
    xp[i] = ONE + radius + (i % 6) * 2 * radius;
    yp[i] = ONE + radius + (i / 6) * 2 * radius;
  }

  // Timer0 already interrupts every ~1 ms for millis(); this asks it to run
  // ISR(TIMER0_COMPA_vect) too, halfway through each count
  OCR0A = 128;
  bitSet(TIMSK0, OCIE0A);
}

void loop()
{
  // at most 25 steps a second
  static unsigned long lastStep = 0;
  if (millis() - lastStep < 40)
    return;
  lastStep = millis();

  readTilt();
  step();
  draw();
}
