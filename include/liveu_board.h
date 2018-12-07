#define BOARD_LU300_WORKAROUND_VALUE	0xa
#define BOARD_LU610_WORKAROUND_VALUE	0x8

typedef enum {
        BOARD_LU600 = 0x00,
        BOARD_LU300 = 0x0B,
        BOARD_LU610 = 0x09,
}_model_type;

extern _model_type model_type;
