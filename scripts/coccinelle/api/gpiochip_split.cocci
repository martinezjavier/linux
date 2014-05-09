// GPIO subsystem refactoring
//
// Confidence: Low
// Copyright (C) 2014 Javier Martinez Canillas.  GPLv2.
// URL: http://coccinelle.lip6.fr/
// Options: --include-headers

// Base rule
@r@
identifier i, ops;
@@

struct gpio_chip i = {...};

@script:python b@
i << r.i;
j;
@@

coccinelle.j = i + "_ops"

@r1@
identifier r.i, b.j;
@@

+ struct gpio_chip_ops j = {};
+
struct gpio_chip i = {
...,
+ .ops = &j,
};

// .request
@r2@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.request = pfn,
...,
};

@@
identifier r.i, r2.pfn;
@@

struct gpio_chip i = {
...,
-	.request = pfn,
...,
};

// .free
@r3@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.free = pfn,
...,
};

@@
identifier r.i, r3.pfn;
@@

struct gpio_chip i = {
...,
-	.free = pfn,
...,
};

// .get_direction
@r4@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.get_direction = pfn,
...,
};

@@
identifier r.i, r4.pfn;
@@

struct gpio_chip i = {
...,
-	.get_direction = pfn,
...,
};

// .direction_input
@r5@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.direction_input = pfn,
...,
};

@@
identifier r.i, r5.pfn;
@@

struct gpio_chip i = {
...,
-	.direction_input = pfn,
...,
};

// .direction_output
@r6@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.direction_output = pfn,
...,
};

@@
identifier r.i, r6.pfn;
@@

struct gpio_chip i = {
...,
-	.direction_output = pfn,
...,
};

// .get
@r7@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.get = pfn,
...,
};

@@
identifier r.i, r7.pfn;
@@

struct gpio_chip i = {
...,
-	.get = pfn,
...,
};

// .set
@r8@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.set = pfn,
...,
};

@@
identifier r.i, r8.pfn;
@@

struct gpio_chip i = {
...,
-	.set = pfn,
...,
};

// .set_debounce
@r9@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.set_debounce = pfn,
...,
};

@@
identifier r.i, r9.pfn;
@@

struct gpio_chip i = {
...,
-	.set_debounce = pfn,
...,
};

// .dbg_show
@r10@
identifier r.i, pfn;
@@

struct gpio_chip i = {
...,
	.dbg_show = pfn,
...,
};

@@
identifier r.i, r10.pfn;
@@

struct gpio_chip i = {
...,
-	.dbg_show = pfn,
...,
};
