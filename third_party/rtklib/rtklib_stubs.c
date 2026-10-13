/* SPDX-License-Identifier: BSD-2-Clause
 *
 * Application-side symbols that RTKLIB declares in rtklib.h but expects the
 * host application to define (they live in the CUI/GUI apps upstream):
 *   showmsg()  - progress/message callback used by readrnxt/preceph/sbas/rtkcmn
 *   settspan() - time-span callback for the GUI progress bar
 *   settime()  - current-time callback for the GUI progress bar
 * livo_gnss never shows messages, so these are no-ops (DESIGN §12.1 "应用桩").
 *
 * sofa.c (IAU SOFA derivative, different license) is NOT compiled. RTKLIB only
 * references it under SUNPOS_SOFA / MOONPOS_SOFA, which this build does not
 * define, so no stub is required for it (rtkcmn.c:3970/:4022).
 */
#include <stdarg.h>
#include <stdio.h>
#include "rtklib.h"

extern int showmsg(const char *format, ...)
{
  (void)format;
  return 0;
}

extern void settspan(gtime_t ts, gtime_t te)
{
  (void)ts;
  (void)te;
}

extern void settime(gtime_t time)
{
  (void)time;
}

/* Receiver decoders that rcvraw.c input_raw()/input_rawf() dispatch to but that this build does not
 * compile: rcv/comnav.c no longer compiles in RTKLIB-EX v2.5.1 (references lam_carr and nav_t.leaps,
 * both removed upstream; the upstream CMake does not build it either) and rcv/tersus.c is likewise
 * not part of the upstream library. Returning 0 means "no complete message" for those formats. */
extern int input_cnav(raw_t *raw, uint8_t data)
{
  (void)raw;
  (void)data;
  return 0;
}

extern int input_cnavf(raw_t *raw, FILE *fp)
{
  (void)raw;
  (void)fp;
  return 0;
}

extern int input_tersus(raw_t *raw, uint8_t data)
{
  (void)raw;
  (void)data;
  return 0;
}

extern int input_tersusf(raw_t *raw, FILE *fp)
{
  (void)raw;
  (void)fp;
  return 0;
}
