/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/
#include <rpc/msgpack/predef/detail/test.h>

#ifndef MSGPACK_PREDEF_MAKE_H
#define MSGPACK_PREDEF_MAKE_H

/*
Shorthands for the common version number formats used by vendors...
*/

/*`
[heading `MSGPACK_PREDEF_MAKE_..` macros]

These set of macros decompose common vendor version number
macros which are composed version, revision, and patch digits.
The naming convention indicates:

* The base of the specified version number. "`MSGPACK_PREDEF_MAKE_0X`" for
  hexadecimal digits, and "`MSGPACK_PREDEF_MAKE_10`" for decimal digits.
* The format of the vendor version number. Where "`V`" indicates the version digits,
  "`R`" indicates the revision digits, "`P`" indicates the patch digits, and "`0`"
  indicates an ignored digit.

Macros are:
*/
/*` `MSGPACK_PREDEF_MAKE_0X_VRP(V)` */
#define MSGPACK_PREDEF_MAKE_0X_VRP(V) MSGPACK_VERSION_NUMBER((V&0xF00)>>8,(V&0xF0)>>4,(V&0xF))
/*` `MSGPACK_PREDEF_MAKE_0X_VVRP(V)` */
#define MSGPACK_PREDEF_MAKE_0X_VVRP(V) MSGPACK_VERSION_NUMBER((V&0xFF00)>>8,(V&0xF0)>>4,(V&0xF))
/*` `MSGPACK_PREDEF_MAKE_0X_VRPP(V)` */
#define MSGPACK_PREDEF_MAKE_0X_VRPP(V) MSGPACK_VERSION_NUMBER((V&0xF000)>>12,(V&0xF00)>>8,(V&0xFF))
/*` `MSGPACK_PREDEF_MAKE_0X_VVRR(V)` */
#define MSGPACK_PREDEF_MAKE_0X_VVRR(V) MSGPACK_VERSION_NUMBER((V&0xFF00)>>8,(V&0xFF),0)
/*` `MSGPACK_PREDEF_MAKE_0X_VRRPPPP(V)` */
#define MSGPACK_PREDEF_MAKE_0X_VRRPPPP(V) MSGPACK_VERSION_NUMBER((V&0xF000000)>>24,(V&0xFF0000)>>16,(V&0xFFFF))
/*` `MSGPACK_PREDEF_MAKE_0X_VVRRP(V)` */
#define MSGPACK_PREDEF_MAKE_0X_VVRRP(V) MSGPACK_VERSION_NUMBER((V&0xFF000)>>12,(V&0xFF0)>>4,(V&0xF))
/*` `MSGPACK_PREDEF_MAKE_0X_VRRPP000(V)` */
#define MSGPACK_PREDEF_MAKE_0X_VRRPP000(V) MSGPACK_VERSION_NUMBER((V&0xF0000000)>>28,(V&0xFF00000)>>20,(V&0xFF000)>>12)
/*` `MSGPACK_PREDEF_MAKE_0X_VVRRPP(V)` */
#define MSGPACK_PREDEF_MAKE_0X_VVRRPP(V) MSGPACK_VERSION_NUMBER((V&0xFF0000)>>16,(V&0xFF00)>>8,(V&0xFF))
/*` `MSGPACK_PREDEF_MAKE_10_VPPP(V)` */
#define MSGPACK_PREDEF_MAKE_10_VPPP(V) MSGPACK_VERSION_NUMBER(((V)/1000)%10,0,(V)%1000)
/*` `MSGPACK_PREDEF_MAKE_10_VRP(V)` */
#define MSGPACK_PREDEF_MAKE_10_VRP(V) MSGPACK_VERSION_NUMBER(((V)/100)%10,((V)/10)%10,(V)%10)
/*` `MSGPACK_PREDEF_MAKE_10_VRP000(V)` */
#define MSGPACK_PREDEF_MAKE_10_VRP000(V) MSGPACK_VERSION_NUMBER(((V)/100000)%10,((V)/10000)%10,((V)/1000)%10)
/*` `MSGPACK_PREDEF_MAKE_10_VRPP(V)` */
#define MSGPACK_PREDEF_MAKE_10_VRPP(V) MSGPACK_VERSION_NUMBER(((V)/1000)%10,((V)/100)%10,(V)%100)
/*` `MSGPACK_PREDEF_MAKE_10_VRR(V)` */
#define MSGPACK_PREDEF_MAKE_10_VRR(V) MSGPACK_VERSION_NUMBER(((V)/100)%10,(V)%100,0)
/*` `MSGPACK_PREDEF_MAKE_10_VRRPP(V)` */
#define MSGPACK_PREDEF_MAKE_10_VRRPP(V) MSGPACK_VERSION_NUMBER(((V)/10000)%10,((V)/100)%100,(V)%100)
/*` `MSGPACK_PREDEF_MAKE_10_VRR000(V)` */
#define MSGPACK_PREDEF_MAKE_10_VRR000(V) MSGPACK_VERSION_NUMBER(((V)/100000)%10,((V)/1000)%100,0)
/*` `MSGPACK_PREDEF_MAKE_10_VV00(V)` */
#define MSGPACK_PREDEF_MAKE_10_VV00(V) MSGPACK_VERSION_NUMBER(((V)/100)%100,0,0)
/*` `MSGPACK_PREDEF_MAKE_10_VVRR(V)` */
#define MSGPACK_PREDEF_MAKE_10_VVRR(V) MSGPACK_VERSION_NUMBER(((V)/100)%100,(V)%100,0)
/*` `MSGPACK_PREDEF_MAKE_10_VVRRPP(V)` */
#define MSGPACK_PREDEF_MAKE_10_VVRRPP(V) MSGPACK_VERSION_NUMBER(((V)/10000)%100,((V)/100)%100,(V)%100)
/*` `MSGPACK_PREDEF_MAKE_10_VVRR0PP00(V)` */
#define MSGPACK_PREDEF_MAKE_10_VVRR0PP00(V) MSGPACK_VERSION_NUMBER(((V)/10000000)%100,((V)/100000)%100,((V)/100)%100)
/*` `MSGPACK_PREDEF_MAKE_10_VVRR0PPPP(V)` */
#define MSGPACK_PREDEF_MAKE_10_VVRR0PPPP(V) MSGPACK_VERSION_NUMBER(((V)/10000000)%100,((V)/100000)%100,(V)%10000)
/*` `MSGPACK_PREDEF_MAKE_10_VVRR00PP00(V)` */
#define MSGPACK_PREDEF_MAKE_10_VVRR00PP00(V) MSGPACK_VERSION_NUMBER(((V)/100000000)%100,((V)/1000000)%100,((V)/100)%100)
/*`
[heading `MSGPACK_PREDEF_MAKE_*..` date macros]

Date decomposition macros return a date in the relative to the 1970
Epoch date. If the month is not available, January 1st is used as the month and day.
If the day is not available, but the month is, the 1st of the month is used as the day.
*/
/*` `MSGPACK_PREDEF_MAKE_DATE(Y,M,D)` */
#define MSGPACK_PREDEF_MAKE_DATE(Y,M,D) MSGPACK_VERSION_NUMBER((Y)%10000-1970,(M)%100,(D)%100)
/*` `MSGPACK_PREDEF_MAKE_YYYYMMDD(V)` */
#define MSGPACK_PREDEF_MAKE_YYYYMMDD(V) MSGPACK_PREDEF_MAKE_DATE(((V)/10000)%10000,((V)/100)%100,(V)%100)
/*` `MSGPACK_PREDEF_MAKE_YYYY(V)` */
#define MSGPACK_PREDEF_MAKE_YYYY(V) MSGPACK_PREDEF_MAKE_DATE(V,1,1)
/*` `MSGPACK_PREDEF_MAKE_YYYYMM(V)` */
#define MSGPACK_PREDEF_MAKE_YYYYMM(V) MSGPACK_PREDEF_MAKE_DATE((V)/100,(V)%100,1)

#endif
