# Rust Laser GUI (WARNING!!! Vibe Coded project!)

This simple vibe coded project only exists because no other viable alternatives exist for Linux users. This project is completely open source and the author takes NO RESPONSIBILITY of malfunctionings or misuses of the code in this open source repository. Use it at your Own Risk!

The author suggests to test it in some **laser off** runs before using it seriously. Wear Always the best protections while using a Laser! 

## How to build it?

Add the user to the following groups

```
sudo adduser $USER dialout 
```

and

```
sudo adduser $USER tty
```

and type

```
groups
```

To check that the user is inside the groups *dialout* and *tty*.

Then you can test the pre-compiled Appimage (Ubuntu 24.04) version by allowing the execution and then double click. 

Or you can decide to buil it from Rust source (it will takes a minute) using:

```
cargo run
```

This project *does not* generate the gcode itself, it just sends it to the laser engraver.

To generate the gcode file (required) the author has successfully used [LaserWeb 4](https://laserweb.yurl.ch/), which suppports Ubuntu 24.04 (Appimage).

Stay Tuned!

