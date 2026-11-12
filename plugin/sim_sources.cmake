# The plugin's sources for the simulator (GRBLHAL-Sim, built with -DFLIGHT_RECORDER_DIR=<this repository>).
# The board build (PlatformIO) takes every .c here except storage_file.c and board_sim.c: library.json.
set(FLIGHT_RECORDER_SIM_SRC
    "${FLIGHT_RECORDER_DIR}/plugin/flight_recorder.c"
    "${FLIGHT_RECORDER_DIR}/plugin/event_log.c"
    "${FLIGHT_RECORDER_DIR}/plugin/storage.c"
    "${FLIGHT_RECORDER_DIR}/plugin/storage_file.c"
    "${FLIGHT_RECORDER_DIR}/plugin/storage_sd.c"
    "${FLIGHT_RECORDER_DIR}/plugin/board_sim.c"
)
