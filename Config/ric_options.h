#define THROTTLE_INPUT_CHANNEL              CHANNEL_1
#define AILERON_INPUT_CHANNEL               CHANNEL_2
#define ELEVATOR_INPUT_CHANNEL              CHANNEL_3
#define RUDDER_INPUT_CHANNEL                CHANNEL_4
#define MODE_SWITCH_INPUT_CHANNEL           CHANNEL_5
#define BRAKE_THR_SEL_INPUT_CHANNEL         CHANNEL_UNUSED
#define BRAKE_INPUT_CHANNEL                 CHANNEL_UNUSED
#define FLAPS_INPUT_CHANNEL                 CHANNEL_6
#define CAMERA_PITCH_INPUT_CHANNEL          CHANNEL_UNUSED
#define CAMERA_YAW_INPUT_CHANNEL            CHANNEL_UNUSED
#define CAMERA_MODE_INPUT_CHANNEL           CHANNEL_UNUSED
#define OSD_MODE_SWITCH_INPUT_CHANNEL       CHANNEL_UNUSED
#define RSSI_INPUT_CHANNEL                  CHANNEL_UNUSED
#define MODE_INVERTED_CHANNEL               CHANNEL_UNUSED
#define PASSTHROUGH_A_INPUT_CHANNEL         CHANNEL_9
#define PASSTHROUGH_B_INPUT_CHANNEL         CHANNEL_UNUSED
#define PASSTHROUGH_C_INPUT_CHANNEL         CHANNEL_UNUSED
#define PASSTHROUGH_D_INPUT_CHANNEL         CHANNEL_UNUSED

// NUM_OUTPUTS:
//   NOTE: If USE_PPM_INPUT is enabled above, up to 9 outputs are available.)
// For UDB4/5 boards: Set to 3-8 (or up to 10 using pins RA4 and RA1.)
// For AUAV3 boards:  Set to 3-8 (or up to 11 using pins RE1, RA6 and RA7.)
//                               (this needs developing, so contact the list)
#define NUM_OUTPUTS                         9

// Channel numbers for each output
// Use as is, or edit to match your setup.
//   - Only assign each channel to one output purpose
//   - If you don't want to use an output channel, set it to CHANNEL_UNUSED
//   - If you're set up to use Rudder Navigation (like MatrixNav), then you may want to swap
//     the aileron and runner channels so that rudder is CHANNEL_1, and aileron is 5.
//
// NOTE: If your board is powered from your ESC through the throttle cable, make sure to
// connect THROTTLE_OUTPUT_CHANNEL to one of the built-in Outputs (1, 2, or 3) to make
// sure your board gets power.
//
#define THROTTLE_OUTPUT_CHANNEL             CHANNEL_1
#define AILERON_OUTPUT_CHANNEL              CHANNEL_UNUSED
#define AILERON_SECONDARY_OUTPUT_CHANNEL    CHANNEL_UNUSED
#define ELEVATOR_OUTPUT_CHANNEL             CHANNEL_3
#define RUDDER_OUTPUT_CHANNEL               CHANNEL_4
#define AILERON_LEFT_OUTPUT_CHANNEL         CHANNEL_2
#define FLAP_LEFT_OUTPUT_CHANNEL            CHANNEL_6
#define FLAP_RIGHT_OUTPUT_CHANNEL           CHANNEL_UNUSED
#define AILERON_RIGHT_OUTPUT_CHANNEL        CHANNEL_UNUSED
#define BRAKE_OUTPUT_CHANNEL                CHANNEL_UNUSED
#define FLAPS_OUTPUT_CHANNEL                CHANNEL_UNUSED
#define CAMERA_PITCH_OUTPUT_CHANNEL         CHANNEL_UNUSED
#define CAMERA_YAW_OUTPUT_CHANNEL           CHANNEL_UNUSED
#define TRIGGER_OUTPUT_CHANNEL              CHANNEL_UNUSED
#define PASSTHROUGH_A_OUTPUT_CHANNEL        CHANNEL_9
#define PASSTHROUGH_B_OUTPUT_CHANNEL        CHANNEL_UNUSED
#define PASSTHROUGH_C_OUTPUT_CHANNEL        CHANNEL_UNUSED
#define PASSTHROUGH_D_OUTPUT_CHANNEL        CHANNEL_UNUSED

// Set to 1 to use Output 1 (udb5mini only) for throttle output and Castle Link
// Live data reads to get voltage and current readings from a Castle ESC.
// When set to 1, you should also set THROTTLE_OUTPUT_CHANNEL to 1.
#define USE_CASTLE_LINK_THROTTLE            0


