/* inc/config_ipopt.h.  Generated by configure.  */
/* inc/config_ipopt.h.in.  Generated from configure.ac by autoheader.  */

/* If defined, the Ampl Solver Library is available. */
#define COIN_HAS_ASL 1

/* If defined, the BLAS Library is available. */
#define COIN_HAS_BLAS 1

/* Define to 1 if the Ipopt package is used */
#define COIN_HAS_IPOPT 1

/* If defined, the LAPACK Library is available. */
#define COIN_HAS_LAPACK 1

/* If defined, the MUMPS Library is available. */
#define COIN_HAS_MUMPS 1

/* Define to the debug sanity check level (0 is no test) */
#define COIN_IPOPT_CHECKLEVEL 0

/* Define to the debug verbosity level (0 is no output) */
#define COIN_IPOPT_VERBOSITY 0

/* Define to dummy `main' function (if any) required to link to the Fortran
   libraries. */
/* #undef F77_DUMMY_MAIN */

/* Define to a macro mangling the given C identifier (in lower and upper
   case), which must not contain underscores, for linking with Fortran. */
#define F77_FUNC(name,NAME) NAME

/* As F77_FUNC, but for C identifiers containing underscores. */
#define F77_FUNC_(name,NAME) NAME

/* Define if F77 and FC dummy `main' functions are identical. */
/* #undef FC_DUMMY_MAIN_EQ_F77 */

/* Define to the C type corresponding to Fortran INTEGER */
#define FORTRAN_INTEGER_TYPE int

/* Define to 1 if you have the <assert.h> header file. */
/* #undef HAVE_ASSERT_H */

/* Define to 1 if you have the <cassert> header file. */
#define HAVE_CASSERT 1

/* Define to 1 if you have the <cctype> header file. */
#define HAVE_CCTYPE 1

/* Define to 1 if you have the <cfloat> header file. */
#define HAVE_CFLOAT 1

/* Define to 1 if you have the <cieeefp> header file. */
/* #undef HAVE_CIEEEFP */

/* Define to 1 if you have the <cmath> header file. */
#define HAVE_CMATH 1

/* Define to 1 if you have the <cstdarg> header file. */
#define HAVE_CSTDARG 1

/* Define to 1 if you have the <cstdio> header file. */
#define HAVE_CSTDIO 1

/* Define to 1 if you have the <cstdlib> header file. */
#define HAVE_CSTDLIB 1

/* Define to 1 if you have the <cstring> header file. */
#define HAVE_CSTRING 1

/* Define to 1 if you have the <ctime> header file. */
#define HAVE_CTIME 1

/* Define to 1 if you have the <ctype.h> header file. */
/* #undef HAVE_CTYPE_H */

/* Define to 1 if you have the <dlfcn.h> header file. */
/* #undef HAVE_DLFCN_H */

/* Define to 1 if function drand48 is available */
/* #undef HAVE_DRAND48 */

/* Define to 1 if you have the <float.h> header file. */
/* #undef HAVE_FLOAT_H */

/* Define to 1 if you have the <ieeefp.h> header file. */
/* #undef HAVE_IEEEFP_H */

/* Define to 1 if you have the <inttypes.h> header file. */
/* #undef HAVE_INTTYPES_H */

/* Define to 1 if the linear solver loader should be compiled to allow dynamic
   loading of shared libaries with linear solvers */
#define HAVE_LINEARSOLVERLOADER 1

/* Define to 1 if MA27 is available */
/* #undef HAVE_MA27 */

/* Define to 1 if MA28 is available */
/* #undef HAVE_MA28 */

/* Define to 1 if MA57 is available */
/* #undef HAVE_MA57 */

/* Define to 1 if you have the <math.h> header file. */
/* #undef HAVE_MATH_H */

/* Define to 1 if MC19 is available */
/* #undef HAVE_MC19 */

/* Define to 1 if you have the <memory.h> header file. */
#define HAVE_MEMORY_H 1

/* Define to 1 if Pardiso is available */
/* #undef HAVE_PARDISO */

/* Define to 1 if you are using the parallel version of Pardiso */
/* #undef HAVE_PARDISO_PARALLEL */

/* Define to 1 if function rand is available */
#define HAVE_RAND 1

/* Define to 1 if you have the `snprintf' function. */
/* #undef HAVE_SNPRINTF */

/* Define to 1 if you have the <stdarg.h> header file. */
/* #undef HAVE_STDARG_H */

/* Define to 1 if you have the <stdint.h> header file. */
/* #undef HAVE_STDINT_H */

/* Define to 1 if you have the <stdio.h> header file. */
/* #undef HAVE_STDIO_H */

/* Define to 1 if you have the <stdlib.h> header file. */
#define HAVE_STDLIB_H 1

/* Define to 1 if function std::rand is available */
/* #undef HAVE_STD__RAND */

/* Define to 1 if you have the <strings.h> header file. */
/* #undef HAVE_STRINGS_H */

/* Define to 1 if you have the <string.h> header file. */
#define HAVE_STRING_H 1

/* Define to 1 if you have the <sys/stat.h> header file. */
#define HAVE_SYS_STAT_H 1

/* Define to 1 if you have the <sys/types.h> header file. */
#define HAVE_SYS_TYPES_H 1

/* Define to 1 if you have the <time.h> header file. */
/* #undef HAVE_TIME_H */

/* Define to 1 if you have the <unistd.h> header file. */
/* #undef HAVE_UNISTD_H */

/* Define to 1 if va_copy is avaliable */
/* #undef HAVE_VA_COPY */

/* Define to 1 if you have the <windows.h> header file. */
#define HAVE_WINDOWS_H 1

/* Define to 1 if WSMP is available */
/* #undef HAVE_WSMP */

/* Define to 1 if you have the `_snprintf' function. */
#define HAVE__SNPRINTF 1

/* Define to be the name of C-function for NaNInf check */
#define MY_C_FINITE _finite

/* Name of package */
#define PACKAGE "ipopt"

/* Define to the address where bug reports for this package should be sent. */
#define PACKAGE_BUGREPORT "http://projects.coin-or.org/Ipopt/newticket"

/* Define to the full name of this package. */
#define PACKAGE_NAME "Ipopt"

/* Define to the full name and version of this package. */
#define PACKAGE_STRING "Ipopt 3.5.4"

/* Define to the one symbol short name of this package. */
#define PACKAGE_TARNAME "ipopt"

/* Define to the version of this package. */
#define PACKAGE_VERSION "3.5.4"

/* Set to extension for shared libraries in quotes. */
#define SHAREDLIBEXT "dll"

/* The size of a `int *', as computed by sizeof. */
#define SIZEOF_INT_P 8

/* Define to 1 if you have the ANSI C header files. */
#define STDC_HEADERS 1

/* Version number of package */
#define VERSION "3.5.4"
