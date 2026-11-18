# FLIP Flop

We built a pair of 8x8 LED boards with a FLIP water simulation and shared particles that can be shared with each other!

Project members: Ryan McCormick and Andrew Hurlbut

This was built for Stanford's E40M (Autumn 2026), Lab 3b. It was built in two stages. Firstly, the core simulation was written by hand following our key reference (defined in the header). However, it ran very slow, around 10fps. You can see that code in ./e40m_lab3b_noai. Then, following Horowitz's suggestion of LLMs as an "amazing tool", we wanted to see how far we could extend it even with the tight 3b deadline. 

So, using Claude Opus 5.5 High, we worked through optimizing the code, adding some QoL features, and finally connecting two of the arduinos using the serial connection. The end result is what you see in the demo video below. Pretty neat! The code for the main implementation is in ./e40m_lab3b.

Full-quality demo: [media/demo.mp4](demo.mp4)
