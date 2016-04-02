/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics;

import com.bulletphysics.util.ArrayPool;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

/**
 * Bullet global settings and constants.
 * 
 * @author jezek2
 */
public class BulletGlobals
{

	public static final boolean DEBUG = false;

	//removes all thread local calls, allows non-dynamics stepping threads to ray cast into world
	public static final boolean ALLOW_MULTI_THREAD_WORLD_ACCESS = true;

	// just disables the object pooling system for debug
	// TODO: GImpact dynamic shapes fail when being inited, something in ArrayPool, you get a class cast exception
	// GImpactCollisionAlgorithm.CreateFunc.createCollisionAlgorithm was the thing I played with
	// In the end I think it was just multi threadded acces that was my problem, synch calls added to Object Pool
	public static final boolean DISABLE_POOLING = false;

	public static final float CONVEX_DISTANCE_MARGIN = 0.04f;

	public static final float FLT_EPSILON = 1.19209290e-07f;

	public static final float SIMD_EPSILON = FLT_EPSILON;

	public static final float SIMD_2_PI = 6.283185307179586232f;

	public static final float SIMD_PI = SIMD_2_PI * 0.5f;

	public static final float SIMD_HALF_PI = SIMD_2_PI * 0.25f;

	public static final float SIMD_RADS_PER_DEG = SIMD_2_PI / 360f;

	public static final float SIMD_DEGS_PER_RAD = 360f / SIMD_2_PI;

	public static final float SIMD_INFINITY = Float.MAX_VALUE;

	// for use when threadlocal disabled
	private static BulletGlobals bulletGlobals = new BulletGlobals();

	////////////////////////////////////////////////////////////////////////////

	private static ThreadLocal<BulletGlobals> threadLocal = new ThreadLocal<BulletGlobals>()
	{
		@Override
		protected BulletGlobals initialValue()
		{
			return new BulletGlobals();
		}
	};

	private ContactDestroyedCallback gContactDestroyedCallback;

	private ContactAddedCallback gContactAddedCallback;

	private ContactProcessedCallback gContactProcessedCallback;

	private float contactBreakingThreshold = 0.02f;

	// RigidBody
	private float deactivationTime = 2f;

	private boolean disableDeactivation = false;

	public static ContactAddedCallback getContactAddedCallback()
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			return bulletGlobals.gContactAddedCallback;
		}
		else
		{
			return threadLocal.get().gContactAddedCallback;
		}
	}

	public static void setContactAddedCallback(ContactAddedCallback callback)
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			bulletGlobals.gContactAddedCallback = callback;
		}
		else
		{
			threadLocal.get().gContactAddedCallback = callback;
		}
	}

	public static ContactDestroyedCallback getContactDestroyedCallback()
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			return bulletGlobals.gContactDestroyedCallback;
		}
		else
		{
			return threadLocal.get().gContactDestroyedCallback;
		}
	}

	public static void setContactDestroyedCallback(ContactDestroyedCallback callback)
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			bulletGlobals.gContactDestroyedCallback = callback;
		}
		else
		{
			threadLocal.get().gContactDestroyedCallback = callback;
		}
	}

	public static ContactProcessedCallback getContactProcessedCallback()
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			return bulletGlobals.gContactProcessedCallback;
		}
		else
		{
			return threadLocal.get().gContactProcessedCallback;
		}
	}

	public static void setContactProcessedCallback(ContactProcessedCallback callback)
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			bulletGlobals.gContactProcessedCallback = callback;
		}
		else
		{
			threadLocal.get().gContactProcessedCallback = callback;
		}
	}

	////////////////////////////////////////////////////////////////////////////

	public static float getContactBreakingThreshold()
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			return bulletGlobals.contactBreakingThreshold;
		}
		else
		{
			return threadLocal.get().contactBreakingThreshold;
		}
	}

	public static void setContactBreakingThreshold(float threshold)
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			bulletGlobals.contactBreakingThreshold = threshold;
		}
		else
		{
			threadLocal.get().contactBreakingThreshold = threshold;
		}
	}

	public static float getDeactivationTime()
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			return bulletGlobals.deactivationTime;
		}
		else
		{
			return threadLocal.get().deactivationTime;
		}
	}

	public static void setDeactivationTime(float time)
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			bulletGlobals.deactivationTime = time;
		}
		else
		{
			threadLocal.get().deactivationTime = time;
		}
	}

	public static boolean isDeactivationDisabled()
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			return bulletGlobals.disableDeactivation;
		}
		else
		{
			return threadLocal.get().disableDeactivation;
		}
	}

	public static void setDeactivationDisabled(boolean disable)
	{
		if (ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			bulletGlobals.disableDeactivation = disable;
		}
		else
		{
			threadLocal.get().disableDeactivation = disable;
		}
	}

	////////////////////////////////////////////////////////////////////////////

	/**
	 * Cleans all current thread specific settings and caches.
	 */
	public static void cleanCurrentThread()
	{
		if (!ALLOW_MULTI_THREAD_WORLD_ACCESS)
		{
			threadLocal.remove();
		}
		Stack.libraryCleanCurrentThread();
		ObjectPool.cleanCurrentThread();
		ArrayPool.cleanCurrentThread();
	}

}
