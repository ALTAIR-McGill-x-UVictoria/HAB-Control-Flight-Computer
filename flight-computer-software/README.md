# Software Information

This project created is using PlatformIO in VS Code to compile and upload code to the Teensy 4.1.

# Run Instructions

## Main Program
To run the main program `src/main.cpp`, the `main` or default environment can be selected. Then, the program can be compiled and uploaded to the board with button in the VS Code UI or the following command:

```pio run -t upload```

## Test Programs

Test programs can be created in the `test` directory. They should each be created in `.cpp` files each within their own directory with the same name starting with `test_`. For example, `test/test_sensors_output/test_sensors_output.cpp`.

### Option 1:
To run a test program, the following command can be used from any environment:

```pio test --without-testing -vv -e test -f <TEST_NAME>```

Example:

```pio test --without-testing -vv -e test -f test_sensors_output```

This command uses the `test` environment written in `platformio.ini`, but can be run from any selected environment.

### Option 2:
If preferred, an environment can be created in `platformio.ini` for each test program. This environment can then be selected, allowing for the corresponding test program to run as a regular program.