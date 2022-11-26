/* Ansteuerung für EINE serielle Conrad-Relaisplatine 8fa Best. Nr. 96 77 20

Original Source: http://www.netzmafia.de/skripten/hardware/relais/relais.html

Another program for multiple 8fa cards on one port: http://www.relaiskarte.thomas-dohl.de/

 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.


compile e. g. with
gcc -O3 -lpthread -o relais relais.c


Usage: relais <device name> <Parameter>

Example: relais /dev/ttyS0 -on


Rolf Freitag 2005: Added select, tcflush, timeout with pthread, ioctl for exclusive mode,
                   time checks, packet length checks, check of the address, printing of
                   the firmware version, pthread_join, 200 ms pause, signal handler,
                   option for the device file (e. g. /dev/ttyS0), ...
2004-7-21

 */

//#define DEBUG

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>              /* File control definitions */
#include <termios.h>            /* POSIX terminal control definitions */
#include <pthread.h>
#include <iso646.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <setjmp.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>             // signals

/*
 * Befehlssatz der Karte:
 * <x> = 'dont't care', in der Regel 0
 * I_XOR = Pruefinfo (XOR aller Werte, wird vom Programm berechnet)
 *
 * Kommando                        Kommandorahmen      Antwort
 * -------------------------------------------------------------------------
 * 0  NO OPERATION - keine Aktion  0 <Adr.> <x> I_XOR    255 <Adr.>  <x> I_XOR
 * 1  SETUP - Initialisierung      1 <Adr.> <x> I_XOR    254 <Adr.> <info> I_XOR
 * 2  GET PORT - Schaltzustand     2 <Adr.> <x> I_XOR    254 <Adr.> <info> I_XOR
 * 3  SET PORT - Relais schalten   3 <Adr.> <data> I_XOR 253 <Adr.> <data> I_XOR
 * 4  GET OPTION - Option lesen    4 <Adr.> <x> I_XOR    254 <Adr.> <info> I_XOR
 * 5  SET OPTION - Option setzen   5 <Adr.> <data> I_XOR 254 <Adr.> <data> I_XOR
 * -------------------------------------------------------------------------
 *
 * Das Init-Kommando initialisiert alle Karten einer Kette. Der Computer
 * erhaelt also bei N Karten N+1 Antworten! Dasselbe gilt fuer
 * das GET PORT-Kommando.
 * Bei den Optionen koennen nur "enable broadcast" und "block broadcast"
 * gesetzt werden:
 *   Wert  Broadcast  Broadcast
 *        ausfuehren  blockieren
 *  ---------------------------------------------
 *    0     nein         nein
 *    1      ja          nein   (Voreinstellung)
 *    2     nein          ja
 *    3      ja           ja    (nicht sinnvoll)
 *  ---------------------------------------------
 * Bei blockierten Brodcast wird bei INIT und GET PORT nur ein NOP an
 * die nachfolgenden Platinen gesendet.
 *
 * Ausgangsport ist normalerweise ttS0 bis ttyS3.
 * Der Port muss fuer den user oder die Gruppe, unter der
 * das Programm laeuft, Schreibberechtigung haben.
 * Die User des Programms koennen in die Gruppe von ttySx
 * aufgenommen werden (meist 'tty' oder 'dialout').
 * Gegebenfalls kann man auch das Programm der Gruppe von ttySx
 * zuordnen und das SetGroupId-Bit setzen.
 */

// show conrad bugs
#define IGNORE_CONRAD_BUGS

// repeat till success
#define EXIT_ONLY_ON_SUCCESS

volatile static int g_fd;       // global file descriptor (for device file)
volatile static long long int g_lli_time0, g_lli_time1, g_lli_time2, g_lli_time3;       // global times
static pthread_t thread1;       // open thread
static char *a_device;      // device name

// options
static int opt_e;
static const char *opt_s;


// time(NULL) with microsecond resolution
inline signed long long int
get_time (void)
{
  struct timeval ti;
  struct timezone tzp;

  gettimeofday (&ti, &tzp);
  return (ti.tv_usec + 1000000 * ((long long int) ti.tv_sec));
}

void
message (const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  if (opt_e)
    vprintf(fmt, ap);
  va_end(ap);
}

int
open_port (void)
{
  /*
   * Oeffnet seriellen Port
   * Gibt das Filehandle zurueck oder -1 bei Fehler
   * der Parameter port muss 0, 1, 2 oder 3 sein
   *
   * RS232-Parameter
   * - 19200 baud
   * - 8 bits/byte
   * - no parity
   * - no handshake
   * - 1 stop bit
   */
  int fd, modelines = 0;
  struct termios options;

/*
  message ("init port\n");
  switch (port)
  {
    case 0:
      fd = open ("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
      break;
    case 1:
      fd = open ("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
      break;
    case 2:
      fd = open ("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
      break;
    case 3:
      fd = open ("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY);
      break;
    case 10:
      fd = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
      break;
    case 11:
      fd = open ("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
      break;
    default:
      fd = -1;
      break;
  }
*/
  fd = open (a_device, O_RDWR | O_NOCTTY | O_NDELAY);
  message ("got file descriptor fd=%d\n", fd);
  if (fd >= 0)
  {
    if (!isatty (fd))
    {
      fprintf (stderr, "The Device %s is no Terminal-Device !\n", a_device);
      return (-1);
    }
    /* get the current options */
    fcntl (fd, F_SETFL, 0);
    if (tcgetattr (fd, &options) != 0)
      return (-1);

    cfsetispeed (&options, B19200);     /* setze 19200 bps */
    cfsetospeed (&options, B19200);
    /* setze Optionen */
    options.c_cflag &= ~PARENB; /* kein Paritybit */
    options.c_cflag &= ~CSTOPB; /* 1 Stopbit */
    options.c_cflag &= ~CSIZE;  /* 8 Datenbits */
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);        /* CD-Signal ignorieren */
    /* Kein Echo, keine Steuerzeichen, keine Interrupts */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;  /* setze "raw" Input */
    options.c_cc[VMIN] = 0;     /* warten auf min. 0 Zeichen */
    options.c_cc[VTIME] = 10;   /* Timeout 10 Sekunden */
#ifdef DEBUG
    printf ("before tcflush\n");
#endif
    tcflush (fd, TCIOFLUSH);
#ifdef DEBUG
    printf ("before tcsetattr\n");
#endif
    if (-1 == tcsetattr (fd, TCSAFLUSH, &options) != 0)
    {
      fprintf (stderr, "ERROR: Could not set terminal attributes for %s !\n", a_device);
      return (-1);
    }
#ifdef DEBUG
    printf ("before ioctl\n");
#endif
    // Set IO-Control-Parameter and check the device.
    if (-1 == ioctl (fd, TIOCEXCL, &modelines)) // Put the tty into exclusive mode.
    {
      fprintf (stderr, "ERROR at ioctl TIOCEXCL on port %s!\n", a_device);
      perror ("ioctl()");
      return (-1);
    }
  }
  else
  {
    fprintf (stderr, "Could not open device %s.\n", a_device);
  }
  return (fd);
}


int
sndcmd (const int fd, const unsigned char command, const unsigned char addr, const unsigned char data)
{
  /*
   * Sendet Kommando an die Relaiskarte
   * die Berechnung der Pruefsumme erfolgt automatisch
   * Rueckgabe: 0 bei Erfolg, -1 bei Fehler
   */
  unsigned char wbuf[4];
  int i;

  g_lli_time0 = get_time ();        // store start time
  wbuf[0] = command;
  wbuf[1] = addr;
  wbuf[2] = data;
  /* Pruefsumme */
  wbuf[3] = wbuf[0] ^ wbuf[1] ^ wbuf[2];
#ifdef DEBUG
  printf (" -> send: %d %d %d %d\n", wbuf[0], wbuf[1], wbuf[2], wbuf[3]);
#endif
  usleep (100000L);             // 100 ms Pause, Pause für die Karte
  i = write (fd, wbuf, 4);
  tcdrain (fd);                 // wait till the output has been transmitted
  usleep (10000L);             // 10 ms Pause, Pause für die Karte
  g_lli_time1 = get_time ();        // store send time
  if (4 == i)
    return (0);
  message ("ERROR: Could send only %d of 4 Bytes\n", i);
  return (-1);
}


int
rcvstat (const int fd, unsigned char *answer, unsigned char *addr, unsigned char *data)
{
  /*
   * Empfaengt Status von der Relaiskarte
   * die Berechnung der Pruefsumme erfolgt automatisch
   * Rueckgabe: 0 bei Erfolg, -1 bei Fehler
   */
  unsigned char rbuf[4] = { 0 };
  int i_xor, i_retval, i=0;
  long long int lli_timediff0, lli_timediff1;
  struct timeval timeout;
  fd_set readfs;                // file descriptor set

 loop:
  memset (&timeout, 0, sizeof (timeout));
  timeout.tv_sec = 1;           // 1 second timeout for select
  FD_ZERO (&readfs);
  FD_SET (fd, &readfs);
  i_retval = select (fd + 1, &readfs, NULL, NULL, &timeout);
  if (0 != i_retval)
  {
    if (i_retval == -1)
    {
      perror ("select()");
      tcflush (fd, TCIOFLUSH);  // flush buffers
      return (-1);
    }
    g_lli_time2 = get_time ();      // store first receive time
    i += read (fd, rbuf, 4);
    if (i < 4) // not all bytes have been received: loop
	goto loop;
    g_lli_time3 = get_time ();      // store last receive time
    lli_timediff0 = g_lli_time3 - g_lli_time0;  // first receive time - send time
    lli_timediff1 = g_lli_time3 - g_lli_time1;  // last receive time - send time
    if ((lli_timediff0 < 5000) or (lli_timediff1 > 25000))      // < 5 ms or > 25 ms
    {
      message ("frame warning\n");
      // return (-1);              // impossible short time or extrem long time
    }
    i_xor = rbuf[0] ^ rbuf[1] ^ rbuf[2];
#ifdef DEBUG
    printf (" -> recv: %d %d %d %d\n", rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
#endif
    *answer = rbuf[0];
    *addr = rbuf[1];
    *data = rbuf[2];
    if ((i_xor == rbuf[3]) and (4 == i) and (1 == *addr))       // checksum of 4 bytes and addr ok
      return (0);
    else
    {
      i_retval = 0;
      if (i_xor != rbuf[3])
      {
        message ("ERROR: XOR checksum is %x but should be %x.\n", i_xor, rbuf[3]);
        i_retval--;
      }
      if (4 != i)
      {
        message ("ERROR: Could receive only %d of 4 Bytes.\n", i);
        i_retval--;
      }
      if (1 != *addr)
      {
#ifndef IGNORE_CONRAD_BUGS
        message ("ERROR: Receive address is %d but should be 1, ignoring.\n", *addr);
#endif
        // do not set i_retval: ignore the old conrad bug
      }
      return (i_retval);
    }
  }                             // if (0 != i_retval)
  fprintf (stderr, "Connection timeout (select failed).\n");
  return (-1);
}


void
help (void)
{
  /* Hilfetext ausgeben */
  fprintf (stderr, "Ansteuerprogramm fuer die serielle Relaiskarte von Conrad\n");
  fprintf (stderr, "No Parameters!\n");
  fprintf (stderr, "\n");
  fprintf (stderr, "Usage: relais <device name> <Parameter>\n");
  fprintf (stderr, "\n");
  fprintf (stderr, "Parameter:\n");
  fprintf (stderr, "   -stat:  Status der Relais als Dezimalzahl\n");
  fprintf (stderr, "           Bit=1: Relais an, Bit=0: Relais aus\n");
  fprintf (stderr, "           keine weiteren Parameter moeglich\n");
  fprintf (stderr, "   -off:   alle Relais aus\n");
  fprintf (stderr, "   -on:    alle Relais an\n");
  fprintf (stderr, "   -sx:    Relais x einschalten (1 <= x <= 8)\n");
  fprintf (stderr, "   -rx:    Relais x ausschalten (1 <= x <= 8)\n");
  fprintf (stderr, "\n");
  fprintf (stderr, "Beispiel:\n");
  fprintf (stderr, "   relais /dev/ttyUSB0 -off -s1 -s3: Relais 1 und 3 einschalten\n");
  fprintf (stderr, "   relais /dev/ttyUSB0 -s4 -r3:      Relais 4 ein- und 3 ausschalten\n");
  fprintf (stderr, "   relais /dev/ttyUSB0 -on -r6:      alle Relais ausser 6 einschalten\n");
  fprintf (stderr, "\n");
  fprintf (stderr, "Rueckgabewert:\n");
  fprintf (stderr, "OK, Kontaktstellung: <Kontaktstellung dezimal>\n");
  fprintf (stderr, "z. B. 'OK, Kontaktstellung: 5' --> Relais 1 und 3 on\n");
  fprintf (stderr, "Bei Fehler wird FAIL: und der komplette Status\n");
  fprintf (stderr, "zurueckgegeben (Antwortcode Adresse Daten/Info).\n");
  fprintf (stderr, "\n");
}


void
sig_handler_main (const int sig)        // signal handler
{
  if ((SIGINT == sig) or (SIGILL == sig) or (SIGKILL == sig) or (SIGSEGV == sig) or (SIGTERM == sig))
  {
    (void) fprintf (stderr, "\a\a\a\a\a\a\a Signal %d, program exiting... \r\n\a\a\a", sig);
    exit (-sig);
  }
  return;
}


void
sig_handler1 (const int sig)    // signal handler for the open thread
{
  static int retval;

  retval = -sig;
  if (SIGUSR1 == sig)
  {
    pthread_exit ((void *) &retval);
  }
  if ((SIGINT == sig) or (SIGILL == sig) or (SIGKILL == sig) or (SIGSEGV == sig) or (SIGTERM == sig))
  {
    (void) fprintf (stderr, "\a\a\a\a\a\a\a Signal %d, program exiting... \r\n\a\a\a", sig);
    exit (retval);
  }
  return;
}


// simple timeout
void *
func_timeout (void *threadid __attribute__((unused)))
{
  static int retval;            // Without static the returned retval would always be zero.
  int i;

  retval = 0;
  for (i = 0; i <= 0xff; i++)
    signal (i, sig_handler_main);
  sleep (1);                    // timeout in s, deadline for open thread
  pthread_kill (thread1, SIGUSR1);
  if (0 >= g_fd)                // device file could not be opened
  {
    message ("ERROR: Timeout, device file could not be opened, file descriptor is %d.\n", g_fd);
    retval = -1;
  }
  pthread_exit ((void *) &retval);
}


// open thread which can hang at open_port
void *
func_open (void *threadid __attribute__((unused)))
{
  static int retval;            // Without static the returned retval would always be zero.
  int i;

  retval = 0;
  for (i = 0; i <= 0xff; i++)
    signal (i, sig_handler1);
  if (-1 == (g_fd = open_port ()))
  {
    fprintf (stderr, "Cannot open %s\n", a_device);
    retval = -1;
  }
#ifdef DEBUG
  printf ("g_fd=%d\n", g_fd);
#endif
  pthread_exit ((void *) &retval);
}

unsigned
option_s (unsigned val)
{
  // decode -s string
  // format: 01xx10 where
  // 0: clear bit
  // 1: set bit
  // x: ignore
  unsigned value = val, bitval = 1;
  size_t optlen = strlen(opt_s);
  for (int i = 0; i < 8; i++)
  {
    if (optlen == 0)
      break;
    optlen--;
    char c = opt_s[optlen];
    switch (c)
    {
    case '1':
      value |= bitval;
      break;

    case '0':
      value &= ~bitval;
      break;

    case 'x':
      // do nothing here, just shift
      break;

    default:
      fprintf (stderr, "ERROR: unknown character in -s option: %c\n", c);
    }
    bitval <<= 1;
  }
  if (optlen > 0)
    fprintf (stderr, "WARNING: -s option string too long\n");

  return value;
}

int
main (int argc, char *argv[])
{                               // static asserts that all values are initialized with 0 recursively
  int i, i_ret, id0, id1;        // i, File descriptor for the port, return value, thread id
  unsigned char ans, adr, stat, val, rval = 0, retval;
  pthread_attr_t attr;
  int thread_return0, thread_return1;    // for pthread_return
  void *statusp;         // for pthread_return
  jmp_buf env;           // environment
  pthread_t thread0;

  setjmp (env);                 // restart point after failure
  for (i = 0; i <= 0xff; i++)
    signal (i, sig_handler_main);
  while ((i = getopt(argc, argv, "d:ehir:s:")) != -1) {
    switch (i) {
    case 'd':
      a_device = optarg;
      break;

    case 'e':
      // verbose error messages
      opt_e = 1;
      break;

    case 'h':
      help();
      exit(0);
      break;

    case 'i':
      // initialize card; ignored
      break;

    case 'r':
      // card number; ignored
      break;

    case 's':
      // set/clear string
      opt_s = optarg;
      break;

    default:
      fprintf(stderr, "unknown option %c\n", i);
      help();
      exit(1);
    }
  }
  argc -= optind;
  argv += optind;
  if (argc != 0) {
    fprintf(stderr, "extraneous arguments, starting at %s\n", argv[0]);
    exit(1);
  }
  if (a_device == NULL) {
    fprintf(stderr, "no device name\n");
    exit(1);
  }
  if (pthread_attr_init (&attr))
  {
    fprintf (stderr, "pthread_attr_init FAILED\n");
    goto err_end;
  }
  if (pthread_attr_setdetachstate (&attr, PTHREAD_CREATE_JOINABLE))
  {
    fprintf (stderr, "\npthread_attr_setdetachstate FAILED\n");
    (void) pthread_attr_destroy (&attr);
    goto err_end;
  }
  // create thread for timeout (to avoid hangup in open_serial, e. g. when the device can not be opened)
  if ((i_ret = pthread_create (&thread0, &attr, func_timeout, (void *) &id0)))
  {
    fprintf (stderr, "ERROR; return code from pthread_create(...) is %d\n", i_ret);
    perror ("pthread_create(...)\n");
    goto err_end;
  }
  if ((i_ret = pthread_create (&thread1, &attr, func_open, (void *) &id1)))
  {
    fprintf (stderr, "ERROR; return code from pthread_create(...) is %d\n", i_ret);
    perror ("pthread_create(...)\n");
    goto err_end;
  }
  (void) pthread_join (thread0, &statusp);
  thread_return0 = *(int *) statusp;
  (void) pthread_join (thread1, &statusp);
  thread_return1 = *(int *) statusp;
  if (thread_return0 or thread_return1) // if timeout
    goto err_end;
  // Karte Initialisieren
  sndcmd (g_fd, 1, 1, 0);
  val = rcvstat (g_fd, &ans, &adr, &stat);
  if (val or (ans != 254))
  {
    fprintf (stderr, "FAIL: card init failed (%d %d %d %d).\n", ans, adr, rval, val);
    sleep (1);                  // wait till the card wakes up ...
    goto err_end;
  }
  message ("Firmware-Version %hhu\n", stat);
  // zweite Antwort auf das init vernichten
  val = rcvstat (g_fd, &ans, &adr, &rval);
  if (val or (ans != 1))
  {
    fprintf (stderr, "FAIL: second part of card init failed (%d %d %d %d).\n", ans, adr, rval, val);
    sleep (1);                  // wait till the card wakes up ...
    goto err_end;
  }

  /* Aktuellen Stand abfragen */
  sndcmd (g_fd, 2, 1, 0);
  val = rcvstat (g_fd, &ans, &adr, &stat);
  if ((val) or (ans != 253))
  {
    fprintf (stderr, "FAIL: Could not read the status (%d %d %d %d).\n", ans, adr, rval, val);
    sleep (1);                  // wait till the card wakes up ...
    goto err_end;
  }
  message ("OK, Aktuelle Kontaktstellung: %d\n", stat);

  /* Parameter auswerten */
  if (opt_s)
  {
    rval = option_s(val);
    sndcmd (g_fd, 3, 1, rval);
    val = rcvstat (g_fd, &ans, &adr, &stat);
    if ((val) or (ans != 252))
      {
        message ("FAIL: %d %d %d %d\n", ans, adr, stat, val);
        goto err_end;
      }
    message ("OK, Neue Kontaktstellung: %d\n", stat);
  }
  close (g_fd);
  exit (0);
err_end:
  if (g_fd)
    close (g_fd);
  retval = -1;                  // use another value before goto and delete this line if you need different error codes
#ifdef EXIT_ONLY_ON_SUCCESS
  longjmp (env, retval);
#else
  exit (retval);
#endif
}
