/*
 * videobuf2-fence.h - DMA buffer sharing fence helpers for videobuf 2
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/fence.h>
#include <linux/slab.h>

static DEFINE_SPINLOCK(vb2_fence_lock);

static inline const char *vb2_fence_get_driver_name(struct fence *fence)
{
	return "vb2_fence";
}

static inline const char *vb2_fence_get_timeline_name(struct fence *fence)
{
	return "vb2_fence_timeline";
}

static inline bool vb2_fence_enable_signaling(struct fence *fence)
{
	return true;
}

static const struct fence_ops vb2_fence_ops = {
	.get_driver_name = vb2_fence_get_driver_name,
	.get_timeline_name = vb2_fence_get_timeline_name,
	.enable_signaling = vb2_fence_enable_signaling,
	.wait = fence_default_wait,
};

inline struct fence *vb2_fence_alloc(u64 context, unsigned int seqno)
{
	struct fence *vb2_fence = kzalloc(sizeof(*vb2_fence), GFP_KERNEL);

	if (!vb2_fence)
		return NULL;

	fence_init(vb2_fence, &vb2_fence_ops, &vb2_fence_lock, context, seqno);

	return vb2_fence;
}
