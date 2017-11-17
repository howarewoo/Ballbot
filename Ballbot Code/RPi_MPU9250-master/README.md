RaspberryPi C++ API for Invensense MPU9250 Intertial Measurement Unit.
Requires [WiringPi](http://wiringpi.com/).
Adapted from Bolderflight's Teensy [implementation](https://github.com/bolderflight/MPU9250), q.v. for details.

Quickstart:

<pre>
% sudo make install
</pre>

builds and installs the library <b>libmpu9250.so</b> in <tt>/usr/lib</tt> and the header
<b>libmpu9250.h</b> in <tt>/usr/local/include</tt>.

<pre>
% cd examples/Basic_I2C
% make
% ./Basic_I2C
</pre>

Runs the basic I^2C example.  Same procedure for the basic SPI example.




