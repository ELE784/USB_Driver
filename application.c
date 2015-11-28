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

int main(int argc, char* argv[])
{
  //  FILE *foutput;
  int usbcam;
  int direction_up = 1;
  int direction_down = 2;
  int direction_left = 3;
  int direction_right = 4;
  //  unsigned char * inBuffer;
  //  unsigned char * finalBuf;
  int ret = 0;

  //  inBuffer = malloc((42666)* sizeof(unsigned char));
  //  finalBuf = malloc((42666 * 2)* sizeof(unsigned char));
  //
  //  if((inBuffer == NULL) || (finalBuf == NULL))
  //  {
  //    return -1;
  //  }

  if (argc == 2)
  {
    usbcam = open("/dev/usbcam0", O_RDWR);

    if (strcmp(argv[1], "on") == 0)
    {
      ret = ioctl(usbcam, USBCAM_IOCTL_STREAMON);
      printf("ret = %d with error %d\n", ret, errno);
    }
    else if (strcmp(argv[1], "off") == 0)
    {
      ret = ioctl(usbcam, USBCAM_IOCTL_STREAMOFF);
      printf("ret = %d with error %d\n", ret, errno);
    }
    else if (strcmp(argv[1], "reset") == 0)
    {
      ret = ioctl(usbcam, USBCAM_IOCTL_PANTILT_RESET);
      printf("ret = %d with error %d\n", ret, errno);
    }
    else if (strcmp(argv[1], "up") == 0)
    {
      ret = ioctl(usbcam, USBCAM_IOCTL_PANTILT, &direction_up);
      printf("ret = %d with error %d\n", ret, errno);
    }
    else if (strcmp(argv[1], "down") == 0)
    {
      ret = ioctl(usbcam, USBCAM_IOCTL_PANTILT, &direction_down);
      printf("ret = %d with error %d\n", ret, errno);
    }
    else if (strcmp(argv[1], "left") == 0)
    {
      ret = ioctl(usbcam, USBCAM_IOCTL_PANTILT, &direction_left);
      printf("ret = %d with error %d\n", ret, errno);
    }
    else if (strcmp(argv[1], "right") == 0)
    {
      ret = ioctl(usbcam, USBCAM_IOCTL_PANTILT, &direction_right);
      printf("ret = %d with error %d\n", ret, errno);
    }

    close(usbcam);
  }
  else
  {
    printf("Bad argument\n");
  }
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
