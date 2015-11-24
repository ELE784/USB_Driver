#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "cmd.h"
#include "dht_data.h"

#define DIRECTION_HAUT      1
#define DIRECTION_BAS       2
#define DIRECTION_GAUCHE    3
#define DIRECTION_DROITE    4

int main()
{
  FILE *foutput;
  int usbcam;
  unsigned char * inBuffer;
  unsigned char * finalBuf;
  int ret = 0;

//  inBuffer = malloc((42666)* sizeof(unsigned char));
//  finalBuf = malloc((42666 * 2)* sizeof(unsigned char));
//
//  if((inBuffer == NULL) || (finalBuf == NULL))
//  {
//    return -1;
//  }

  usbcam = open("/dev/usbcam0", O_RDWR);

  //ret = ioctl(usbcam, USBCAM_IOCTL_PANTILT_RESET);

  printf("ret = %d with error %d\n", ret, errno);

  close(usbcam);
  //foutput = fopen("/home/bullshark/C++_workspace/image.jpg", "wb");

  //if(foutput != NULL)
  //{
    // Etape #2
    // Etape #3
    // Etape #4
    // Etape #5
    //memcpy (finalBuf, inBuffer, HEADERFRAME1);
    //memcpy (finalBuf + HEADERFRAME1, dht_data, DHT_SIZE);
    //memcpy (finalBuf + HEADERFRAME1 + DHT_SIZE, inBuffer + HEADERFRAME1, (mySize -HEADERFRAME1));
    //fwrite (finalBuf, mySize + DHT_SIZE, 1, foutput);
    //fclose(foutput);
  //}
}
