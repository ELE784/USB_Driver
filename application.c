
int main()
{
  FILE *foutput;
  unsigned char * inBuffer;
  unsigned char * finalBuf;
  inBuffer = malloc((42666)* sizeof(unsigned char));
  finalBuf = malloc((42666 * 2)* sizeof(unsigned char));
  if((inBuffer == NULL) || (finalBuf == NULL))
  {
    return -1;
  }
  foutput = fopen("/lien/vers/fichier.jpg", "wb");
  if(foutput != NULL)
  {
    // Etape #2
    // Etape #3
    // Etape #4
    // Etape #5
    memcpy (finalBuf, inBuffer, HEADERFRAME1);
    memcpy (finalBuf + HEADERFRAME1, dht_data, DHT_SIZE);
    memcpy (finalBuf + HEADERFRAME1 + DHT_SIZE, inBuffer + HEADERFRAME1, (mySize -HEADERFRAME1));
    fwrite (finalBuf, mySize + DHT_SIZE, 1, foutput);
    fclose(foutput);
  }
}
