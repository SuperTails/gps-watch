# GPS Watch "Landhopper": Firmware
Carnegie Mellon University \
18-500 ECE Design Experience, Spring 2024 \
Gary Bailey, Carson Swoveland, Twain Byrnes

To build, make sure the thumbv7em-none-eabihf target is installed. Deploy with
`cargo run` to use probe-rs, or use `openocd -f openocd.cfg` and `gdb-multiarch -x gdb.cfg`
to enter a debugger.