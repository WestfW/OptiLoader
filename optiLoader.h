#include <stdint.h>

typedef struct image {
    char image_name[30];	       /* Ie "optiboot_diecimila.hex" */
    char image_chipname[12];	       /* ie "atmega168" */
    uint16_t image_chipsig;	       /* Low two bytes of signature */
    uint8_t image_progfuses[5];	       /* fuses to set during programming */
    uint8_t image_normfuses[5];	       /* fuses to set after programming */
    uint8_t image_pagesize;	       /* page size for flash programming */
    char image_hexcode[1500];	       /* intel hex format image (text) */
} image_t;

typedef struct alias {
  char image_chipname[12];
  uint16_t image_chipsig;
  image_t * alias_image;
} alias_t;

#define FUSE_PROT 0			/* memory protection */
#define FUSE_LOW 1			/* Low fuse */
#define FUSE_HIGH 2			/* High fuse */
#define FUSE_EXT 3			/* Extended fuse */

// Forward decl
extern image_t PROGMEM image_328, image_328p, image_168, image_8;
