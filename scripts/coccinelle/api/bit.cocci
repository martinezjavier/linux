// Use the macro BIT() macro if possible
//
// Confidence: High
// Copyright (C) 2014 Javier Martinez Canillas.  GPLv2.
// URL: http://coccinelle.lip6.fr/
// Options: --include-headers

@hasbitops@
@@

#include <linux/bitops.h>

@usesbit@
@@

BIT(...)

@depends on hasbitops && usesbit@
expression E;
@@

- 1 << E
+ BIT(E)

@depends on hasbitops && usesbit@
expression E;
@@

- BIT((E))
+ BIT(E)
