/****************************************************************************
 * ppp_link — NSH command that configures a UART for async PPP and starts
 *            the NuttX netutils/pppd daemon in a dedicated task.
 *
 * Usage (from NSH / rc.board_ppp):
 *   ppp_link start [device] [baud]
 *   ppp_link stop
 *   ppp_link status
 *
 * Defaults: device=/dev/ttyS2  baud=921600
 *
 * What it does:
 *   1. Opens the UART, configures termios for raw 8N1 async PPP:
 *        - requested baud rate via cfsetspeed()
 *        - no hardware flow control (CRTSCTS cleared)
 *        - raw mode (no echo, no signal chars)
 *   2. Closes the fd — NuttX UART drivers persist termios state on the
 *      device node so pppd() inherits the baud/mode when it reopens.
 *   3. Spawns a background task that calls pppd() (NuttX netutils/pppd).
 *      pppd() runs async HDLC framing (RFC 1662, ahdlc.c) over the raw
 *      byte stream and brings up ppp0 via LCP/IPCP negotiation.
 *      It blocks until the link drops.
 *
 * IP address negotiation (local=10.0.0.1 remote=10.0.0.2) is handled by
 * IPCP — no extra arguments to pppd() are needed.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <netutils/pppd.h>

/* ── tuneable defaults ──────────────────────────────────────────────── */

#define PPP_DEFAULT_TTY        "/dev/ttyS2"
#define PPP_DEFAULT_BAUD       115200
#define PPP_TASK_STACK_SIZE    4096
#define PPP_TASK_PRIORITY      50   /* below normal flight-control tasks */

/* ── internal state ─────────────────────────────────────────────────── */

static pid_t g_pppd_pid    = -1;
static char  g_ttyname[32] = PPP_DEFAULT_TTY;

/* ── helpers ─────────────────────────────────────────────────────────── */

static speed_t baud_to_speed(uint32_t baud)
{
  switch (baud)
    {
      case 9600:   return B9600;
      case 19200:  return B19200;
      case 38400:  return B38400;
      case 57600:  return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default:     return B921600;
    }
}

/**
 * configure_tty — apply async-PPP termios then close.
 * NuttX UART drivers preserve termios state across open/close on the same
 * device node, so pppd() will find the UART at the right baud when it opens.
 */
static int configure_tty(const char *dev, uint32_t baud)
{
  struct termios tio;
  int fd;
  int ret;

  fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "ppp_link: cannot open %s: %s\n", dev, strerror(errno));
      return -errno;
    }

  ret = tcgetattr(fd, &tio);
  if (ret < 0)
    {
      fprintf(stderr, "ppp_link: tcgetattr failed: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  /* Raw mode — no echo, no signal chars, no canonical processing */
  cfmakeraw(&tio);

  /* 8 data bits, no parity, 1 stop bit */
  tio.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
  tio.c_cflag |= CS8 | CREAD | CLOCAL;

  /* No hardware flow control (STM32 CTS/RTS unreliable at high baud rates) */
  tio.c_cflag &= ~CRTSCTS;

  /* Baud rate */
  cfsetspeed(&tio, baud_to_speed(baud));

  /* VMIN=1 VTIME=0: read returns as soon as ≥1 byte is available */
  tio.c_cc[VMIN]  = 1;
  tio.c_cc[VTIME] = 0;

  ret = tcsetattr(fd, TCSANOW, &tio);
  if (ret < 0)
    {
      fprintf(stderr, "ppp_link: tcsetattr failed: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  close(fd);
  return 0;
}

/* ── pppd task entry ─────────────────────────────────────────────────── */

static int pppd_task_main(int argc, FAR char *argv[])
{
  struct pppd_settings_s settings;

  memset(&settings, 0, sizeof(settings));
  strncpy(settings.ttyname, g_ttyname, TTYNAMSIZ - 1);

  /* connect_script/disconnect_script NULL = direct wire, no modem chat */
  settings.connect_script    = NULL;
  settings.disconnect_script = NULL;

  printf("ppp_link: pppd starting on %s (async HDLC / RFC 1662)\n",
         settings.ttyname);

  int ret = pppd(&settings);   /* blocks until link drops */

  printf("ppp_link: pppd exited (%d)\n", ret);
  g_pppd_pid = -1;
  return ret;
}

/* ── entry point ─────────────────────────────────────────────────────── */

__EXPORT int ppp_link_main(int argc, char *argv[]);

int ppp_link_main(int argc, char *argv[])
{
  const char *subcmd = (argc > 1) ? argv[1] : "start";

  /* ── start ── */
  if (strcmp(subcmd, "start") == 0)
    {
      if (g_pppd_pid > 0)
        {
          printf("ppp_link: already running (pid %d)\n", (int)g_pppd_pid);
          return 0;
        }

      const char *dev  = (argc > 2) ? argv[2] : PPP_DEFAULT_TTY;
      uint32_t    baud = (argc > 3) ? (uint32_t)atoi(argv[3]) : PPP_DEFAULT_BAUD;

      strncpy(g_ttyname, dev, sizeof(g_ttyname) - 1);
      g_ttyname[sizeof(g_ttyname) - 1] = '\0';

      printf("ppp_link: configuring %s at %lu baud for async PPP\n",
             dev, (unsigned long)baud);

      if (configure_tty(dev, baud) < 0)
        {
          return -1;
        }

      g_pppd_pid = task_create("pppd_link",
                               PPP_TASK_PRIORITY,
                               PPP_TASK_STACK_SIZE,
                               pppd_task_main,
                               NULL);

      if (g_pppd_pid < 0)
        {
          fprintf(stderr, "ppp_link: task_create failed: %s\n",
                  strerror(errno));
          return -1;
        }

      printf("ppp_link: pppd task started (pid %d)\n", (int)g_pppd_pid);
      return 0;
    }

  /* ── stop ── */
  if (strcmp(subcmd, "stop") == 0)
    {
      if (g_pppd_pid <= 0)
        {
          printf("ppp_link: not running\n");
          return 0;
        }

      kill(g_pppd_pid, SIGTERM);
      g_pppd_pid = -1;
      printf("ppp_link: stopped\n");
      return 0;
    }

  /* ── status ── */
  if (strcmp(subcmd, "status") == 0)
    {
      if (g_pppd_pid > 0)
        printf("ppp_link: running on %s (pid %d)\n",
               g_ttyname, (int)g_pppd_pid);
      else
        printf("ppp_link: not running\n");
      return 0;
    }

  printf("Usage: ppp_link {start [device] [baud] | stop | status}\n");
  return 1;
}
