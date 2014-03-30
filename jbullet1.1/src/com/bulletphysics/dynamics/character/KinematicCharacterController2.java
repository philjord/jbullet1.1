package com.bulletphysics.dynamics.character;

import java.util.ArrayList;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.dispatch.LocalPairCachingGhostObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.dynamics.ActionInterface;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;

import cz.advel.stack.Stack;

/**
 * KinematicCharacterController is an object that supports a sliding motion in
 * a world. It uses a {@link GhostObject} and convex sweep test to test for upcoming
 * collisions. This is combined with discrete collision detection to recover
 * from penetrations.<p>
 *
 * Interaction between KinematicCharacterController and dynamic rigid bodies
 * needs to be explicity implemented by the user.
 * 
 * @author tomrbryn
 */
public class KinematicCharacterController2 extends ActionInterface
{

	private static Vector3f[] upAxisDirection = new Vector3f[]
	{ new Vector3f(1.0f, 0.0f, 0.0f), new Vector3f(0.0f, 1.0f, 0.0f), new Vector3f(0.0f, 0.0f, 1.0f), };

	private LocalPairCachingGhostObject ghostObject;

	// is also in ghostObject, but it needs to be convex, so we store it here
	// to avoid upcast
	private ConvexShape convexShape;

	private float verticalVelocity;

	private float verticalOffset;

	private float fallSpeed;

	private float jumpSpeed;

	private float maxSlopeRadians; // Slope angle that is set (used for returning the exact value) 

	private float maxSlopeCosine; // Cosine equivalent of m_maxSlopeRadians (calculated once when set, for optimization)

	private float gravity;

	private float stepHeight;

	private boolean isFreeFly = false;

	private float addedMargin; // TODO: remove this and fix the code

	// this is the desired walk direction, set by the user
	private Vector3f walkDirection = new Vector3f();

	private Vector3f normalizedDirection = new Vector3f();

	// some internal variables
	private Vector3f previousPosition = new Vector3f();

	private Vector3f currentPosition = new Vector3f();

	private float currentStepOffset;

	private Vector3f targetPosition = new Vector3f();

	// NOTE! this is not invloved in the sim, just handed out, so update in order to have the rot handed out
	private Quat4f rotationHolder = new Quat4f(0, 0, 0, 1);

	private Vector3f warpRequest = null;

	// keep track of the contact manifolds
	private ObjectArrayList<PersistentManifold> manifoldArray = new ObjectArrayList<PersistentManifold>();

	private Vector3f touchingNormal = new Vector3f();

	private boolean wasOnGround;

	private boolean useGhostObjectSweepTest;

	private float velocityTimeInterval;

	private int upAxis;

	private ArrayList<CharacterPositionListener> characterPositionListeners = new ArrayList<CharacterPositionListener>();

	//TODO: add a crouch command, slow him down too

	public KinematicCharacterController2(LocalPairCachingGhostObject ghostObject, ConvexShape convexShape, float stepHeight)
	{
		this(ghostObject, convexShape, stepHeight, 1);
	}

	public KinematicCharacterController2(LocalPairCachingGhostObject ghostObject, ConvexShape convexShape, float stepHeight, int upAxis)
	{
		this.upAxis = upAxis;
		this.addedMargin = 0.02f;
		this.walkDirection.set(0, 0, 0);
		this.useGhostObjectSweepTest = true;
		this.ghostObject = ghostObject;
		this.stepHeight = stepHeight;
		this.convexShape = convexShape;
		this.velocityTimeInterval = 0.0f;
		this.verticalVelocity = 0.0f;
		this.verticalOffset = 0.0f;
		this.gravity = 9.8f; // 1G acceleration
		this.fallSpeed = 55.0f; // Terminal velocity of a sky diver in m/s.
		this.jumpSpeed = 10.0f; // ?
		this.wasOnGround = false;
		setMaxSlope((float) ((50.0f / 180.0f) * Math.PI));
	}

	// ActionInterface interface
	public void updateAction(CollisionWorld collisionWorld, float deltaTime)
	{

		// execute a warp request if pending
		if (warpRequest != null)
		{
			Transform xform = Stack.alloc(Transform.class);
			xform.setIdentity();
			xform.origin.set(warpRequest);
			warpRequest = null;
			ghostObject.setWorldTransform(xform);
		}

		if (isFreeFly)
		{
			playerStepFreeFly(deltaTime);
		}
		else
		{
			preStep(collisionWorld);
			playerStep(collisionWorld, deltaTime);
		}

		// update listeners
		synchronized (characterPositionListeners)
		{
			//NOTE good candidate for concurrent mod exceptions here
			for (CharacterPositionListener cpl : characterPositionListeners)
			{
				cpl.positionChanged(currentPosition, rotationHolder);
			}
		}
		previousPosition.set(currentPosition);
	}

	public void setRotation(Quat4f rotation)
	{
		rotationHolder.set(rotation);
	}

	public void addCharacterPositionListener(CharacterPositionListener cpl)
	{
		synchronized (characterPositionListeners)
		{
			if (!characterPositionListeners.contains(cpl))
			{
				characterPositionListeners.add(cpl);
			}
		}
	}

	public void removeCharacterPositionListener(CharacterPositionListener cpl)
	{
		synchronized (characterPositionListeners)
		{
			characterPositionListeners.remove(cpl);
		}
	}

	// ActionInterface interface
	public void debugDraw(IDebugDraw debugDrawer)
	{
	}

	public void setUpAxis(int axis)
	{
		if (axis < 0)
		{
			axis = 0;
		}
		if (axis > 2)
		{
			axis = 2;
		}
		upAxis = axis;
	}

	/**
	 * Caller provides a velocity with which the character should move for the
	 * given time period. After the time period, velocity is reset to zero.
	 * This call will reset any walk direction set by {@link #setWalkDirection}.
	 * Negative time intervals will result in no motion.
	 */

	public void setVelocityForTimeInterval(Vector3f velocity, float timeInterval)
	{
		// gate against crap
		if (Float.isNaN(walkDirection.x))
		{
			velocity.set(0, 0, 0);
		}

		walkDirection.set(velocity);
		normalizedDirection.set(getNormalizedVector(walkDirection, Stack.alloc(Vector3f.class)));
		velocityTimeInterval = timeInterval;
	}

	public void reset()
	{
	}

	/**
	 * NOTE this warp is not done on the currentthread, the warp is done on the next updateAction
	 * @param origin
	 */
	public void warp(Vector3f origin)
	{
		warpRequest = new Vector3f(origin);
	}

	public void preStep(CollisionWorld collisionWorld)
	{
		int numPenetrationLoops = 0;
		while (recoverFromPenetration(collisionWorld))
		{
			numPenetrationLoops++;
			if (numPenetrationLoops > 4)
			{
				//printf("character could not recover from penetration = %d\n", numPenetrationLoops);
				break;
			}
		}

		currentPosition.set(ghostObject.getWorldTransform(Stack.alloc(Transform.class)).origin);
		targetPosition.set(currentPosition);
		//printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);

	}

	public void playerStep(CollisionWorld collisionWorld, float dt)
	{
		//printf("playerStep(): ");
		//printf("  dt = %f", dt);

		// quick check...
		if (velocityTimeInterval > 0.0f)
		{

			wasOnGround = onGround();

			// Update fall velocity.
			verticalVelocity -= gravity * dt;
			if (verticalVelocity > 0.0 && verticalVelocity > jumpSpeed)
			{
				verticalVelocity = jumpSpeed;
			}
			if (verticalVelocity < 0.0 && Math.abs(verticalVelocity) > Math.abs(fallSpeed))
			{
				verticalVelocity = -Math.abs(fallSpeed);
			}
			verticalOffset = verticalVelocity * dt;

			Transform xform = ghostObject.getWorldTransform(Stack.alloc(Transform.class));

			//printf("walkDirection(%f,%f,%f)\n",walkDirection[0],walkDirection[1],walkDirection[2]);
			//printf("walkSpeed=%f\n",walkSpeed);		
			stepUp(collisionWorld);

			//System.out.println("playerStep 4");
			//printf("  time: %f", m_velocityTimeInterval);

			// still have some time left for moving!
			float dtMoving = (dt < velocityTimeInterval) ? dt : velocityTimeInterval;
			velocityTimeInterval -= dt;

			// how far will we move while we are moving?
			Vector3f move = Stack.alloc(Vector3f.class);
			move.scale(dtMoving, walkDirection);

			//printf("  dtMoving: %f", dtMoving);

			// okay, step
			stepHorizontal(collisionWorld, move);

			stepDown(collisionWorld, dt);

			//printf("\n");

			xform.origin.set(currentPosition);
			ghostObject.setWorldTransform(xform);
		}
	}

	public void playerStepFreeFly(float dt)
	{

		// quick check...
		if (velocityTimeInterval > 0.0f)
		{

			// still have some time left for moving!
			float dtMoving = (dt < velocityTimeInterval) ? dt : velocityTimeInterval;
			velocityTimeInterval -= dt;

			// how far will we move while we are moving?
			Vector3f move = Stack.alloc(Vector3f.class);
			move.scale(dtMoving, walkDirection);
			targetPosition.add(currentPosition, move);
			currentPosition.set(targetPosition);
		}

		Transform xform = ghostObject.getWorldTransform(Stack.alloc(Transform.class));
		xform.origin.set(currentPosition);
		ghostObject.setWorldTransform(xform);
	}

	public void setFallSpeed(float fallSpeed)
	{
		this.fallSpeed = fallSpeed;
	}

	public void setJumpSpeed(float jumpSpeed)
	{
		this.jumpSpeed = jumpSpeed;
	}

	public boolean canJump()
	{
		return onGround();
	}

	/**
	 * 0 to1 for a smaller jump great than 1 for bigger
	 * @param multiplier
	 */
	public void jump(float multiplier)
	{
		if (!canJump())
			return;

		verticalVelocity = jumpSpeed * multiplier;

		//#if 0
		//currently no jumping.
		//btTransform xform;
		//m_rigidBody->getMotionState()->getWorldTransform (xform);
		//btVector3 up = xform.getBasis()[1];
		//up.normalize ();
		//btScalar magnitude = (btScalar(1.0)/m_rigidBody->getInvMass()) * btScalar(8.0);
		//m_rigidBody->applyCentralImpulse (up * magnitude);
		//#endif
	}

	public void setGravity(float gravity)
	{
		this.gravity = gravity;
	}

	public float getGravity()
	{
		return gravity;
	}

	public void setMaxSlope(float slopeRadians)
	{
		maxSlopeRadians = slopeRadians;
		maxSlopeCosine = (float) Math.cos(slopeRadians);
	}

	public float getMaxSlope()
	{
		return maxSlopeRadians;
	}

	public boolean onGround()
	{
		return verticalVelocity == 0.0f && verticalOffset == 0.0f;
	}

	// static helper method
	private static Vector3f getNormalizedVector(Vector3f v, Vector3f out)
	{
		out.set(v);
		out.normalize();
		if (out.length() < BulletGlobals.SIMD_EPSILON)
		{
			out.set(0, 0, 0);
		}
		return out;
	}

	protected boolean recoverFromPenetration(CollisionWorld collisionWorld)
	{
		boolean penetration = false;

		collisionWorld.getDispatcher().dispatchAllCollisionPairs(ghostObject.getOverlappingPairCache(), collisionWorld.getDispatchInfo(),
				collisionWorld.getDispatcher());

		currentPosition.set(ghostObject.getWorldTransform(Stack.alloc(Transform.class)).origin);

		float maxPen = 0.0f;
		for (int i = 0; i < ghostObject.getOverlappingPairCache().getNumOverlappingPairs(); i++)
		{
			manifoldArray.clear();

			BroadphasePair collisionPair = ghostObject.getOverlappingPairCache().getOverlappingPairArray().getQuick(i);

			if (collisionPair.algorithm != null)
			{
				collisionPair.algorithm.getAllContactManifolds(manifoldArray);
			}

			for (int j = 0; j < manifoldArray.size(); j++)
			{
				PersistentManifold manifold = manifoldArray.getQuick(j);
				float directionSign = manifold.getBody0() == ghostObject ? -1.0f : 1.0f;
				for (int p = 0; p < manifold.getNumContacts(); p++)
				{
					ManifoldPoint pt = manifold.getContactPoint(p);

					float dist = pt.getDistance();
					if (dist < 0.0f)
					{
						if (dist < maxPen)
						{
							maxPen = dist;
							touchingNormal.set(pt.normalWorldOnB);//??
							touchingNormal.scale(directionSign);
						}
						currentPosition.scaleAdd(directionSign * dist * 0.2f, pt.normalWorldOnB, currentPosition);
						penetration = true;
					}
					else
					{
						//printf("touching %f\n", dist);
					}
				}

				//manifold->clearManifold();
			}
		}

		Transform newTrans = ghostObject.getWorldTransform(Stack.alloc(Transform.class));
		newTrans.origin.set(currentPosition);
		ghostObject.setWorldTransform(newTrans);
		//printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);

		//System.out.println("recoverFromPenetration "+penetration+" "+touchingNormal);

		return penetration;
	}

	protected void stepUp(CollisionWorld world)
	{
		// phase 1: up
		Transform start = Stack.alloc(Transform.class);
		Transform end = Stack.alloc(Transform.class);
		targetPosition.scaleAdd(stepHeight + (verticalOffset > 0.0 ? verticalOffset : 0.0f), upAxisDirection[upAxis], currentPosition);
		start.setIdentity();
		end.setIdentity();

		/* FIXME: Handle penetration properly */
		start.origin.scaleAdd(convexShape.getMargin() + addedMargin, upAxisDirection[upAxis], currentPosition);
		end.origin.set(targetPosition);

		// Find only sloped/flat surface hits, avoid wall and ceiling hits...
		Vector3f up = Stack.alloc(Vector3f.class);
		up.scale(-1f, upAxisDirection[upAxis]);
		KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(ghostObject, up, 0.0f);
		callback.collisionFilterGroup = ghostObject.getBroadphaseHandle().collisionFilterGroup;
		callback.collisionFilterMask = ghostObject.getBroadphaseHandle().collisionFilterMask;

		if (useGhostObjectSweepTest)
		{
			ghostObject.convexSweepTest(convexShape, start, end, callback, world.getDispatchInfo().allowedCcdPenetration);
		}
		else
		{
			// this is a dodgy call that fails on GImapcts see the ghostObject.convexSweepTest above for a better option
			world.convexSweepTest(convexShape, start, end, callback);
		}

		if (callback.hasHit())
		{
			// we moved up only a fraction of the step height
			currentStepOffset = stepHeight * callback.closestHitFraction;
			currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
			verticalVelocity = 0.0f;
			verticalOffset = 0.0f;
		}
		else
		{
			currentStepOffset = stepHeight;
			currentPosition.set(targetPosition);
		}

	}

	protected void updateTargetPositionBasedOnCollision(Vector3f hitNormal)
	{
		updateTargetPositionBasedOnCollision(hitNormal, 0f, 1f);
	}

	protected void updateTargetPositionBasedOnCollision(Vector3f hitNormal, float tangentMag, float normalMag)
	{
		Stack.alloc(Vector3f.class).set(0, 0, 0);
		Vector3f movementDirection = Stack.alloc(Vector3f.class);
		movementDirection.sub(targetPosition, currentPosition);
		float movementLength = movementDirection.length();
		if (movementLength > BulletGlobals.SIMD_EPSILON)
		{
			movementDirection.normalize();

			Vector3f reflectDir = computeReflectionDirection(movementDirection, hitNormal, Stack.alloc(Vector3f.class));
			reflectDir.normalize();

			targetPosition.set(currentPosition);
			if (false) //tangentMag != 0.0)
			{
				Vector3f parComponent = Stack.alloc(Vector3f.class);
				Vector3f parallelDir = parallelComponent(reflectDir, hitNormal, Stack.alloc(Vector3f.class));
				parComponent.scale(tangentMag * movementLength, parallelDir);
				//printf("parComponent=%f,%f,%f\n",parComponent[0],parComponent[1],parComponent[2]);
				targetPosition.add(parComponent);
			}

			if (normalMag != 0.0f)
			{
				Vector3f perpComponent = Stack.alloc(Vector3f.class);
				Vector3f perpindicularDir = perpindicularComponent(reflectDir, hitNormal, Stack.alloc(Vector3f.class));
				perpComponent.scale(normalMag * movementLength, perpindicularDir);
				//printf("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
				targetPosition.add(perpComponent);
			}
		}
		else
		{
			//printf("movementLength don't normalize a zero vector\n");
		}

	}

	protected void stepHorizontal(CollisionWorld collisionWorld, Vector3f walkMove)
	{
		// printf("m_normalizedDirection=%f,%f,%f\n",
		// 	m_normalizedDirection[0],m_normalizedDirection[1],m_normalizedDirection[2]);
		// phase 2: forward and strafe

		Transform start = Stack.alloc(Transform.class);
		Transform end = Stack.alloc(Transform.class);
		targetPosition.add(currentPosition, walkMove);
		start.setIdentity();
		end.setIdentity();

		float fraction = 1.0f;
		Vector3f distance2Vec = Stack.alloc(Vector3f.class);
		distance2Vec.sub(currentPosition, targetPosition);
		float distance2 = distance2Vec.lengthSquared();
		//printf("distance2=%f\n",distance2);

		/*if (touchingContact) {
			if (normalizedDirection.dot(touchingNormal) > 0.0f) {
				updateTargetPositionBasedOnCollision(touchingNormal);
			}
		}*/

		int maxIter = 10;

		while (fraction > 0.01f && maxIter-- > 0)
		{
			start.origin.set(currentPosition);
			end.origin.set(targetPosition);
			KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(ghostObject,
					upAxisDirection[upAxis], -1.0f);
			callback.collisionFilterGroup = ghostObject.getBroadphaseHandle().collisionFilterGroup;
			callback.collisionFilterMask = ghostObject.getBroadphaseHandle().collisionFilterMask;

			float margin = convexShape.getMargin();
			convexShape.setMargin(margin + addedMargin);
			if (useGhostObjectSweepTest)
			{
				ghostObject.convexSweepTest(convexShape, start, end, callback, collisionWorld.getDispatchInfo().allowedCcdPenetration);
			}
			else
			{
				// this is a dodgy call that fails on GImapcts see the ghostObject.convexSweepTest above for a better option
				collisionWorld.convexSweepTest(convexShape, start, end, callback);
			}

			convexShape.setMargin(margin);

			fraction -= callback.closestHitFraction;

			if (callback.hasHit())
			{
				// we moved only a fraction
				Vector3f hitDistanceVec = Stack.alloc(Vector3f.class);
				hitDistanceVec.sub(callback.hitPointWorld, currentPosition);
				//float hitDistance = hitDistanceVec.length();

				// if the distance is farther than the collision margin, move
				//if (hitDistance > addedMargin) {
				//	//printf("callback.m_closestHitFraction=%f\n",callback.m_closestHitFraction);
				//	currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
				//}

				updateTargetPositionBasedOnCollision(callback.hitNormalWorld);

				Vector3f currentDir = Stack.alloc(Vector3f.class);
				currentDir.sub(targetPosition, currentPosition);
				distance2 = currentDir.lengthSquared();
				if (distance2 > BulletGlobals.SIMD_EPSILON)
				{
					currentDir.normalize();
					// see Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners."
					if (currentDir.dot(normalizedDirection) <= 0.0f)
					{
						break;
					}
				}
				else
				{
					//printf("currentDir: don't normalize a zero vector\n");
					break;
				}
			}
			else
			{
				// we moved whole way
				currentPosition.set(targetPosition);
			}

			//if (callback.m_closestHitFraction == 0.f)
			//    break;
		}

	}

	protected void stepDown(CollisionWorld collisionWorld, float dt)
	{

		Transform start = Stack.alloc(Transform.class);
		Transform end = Stack.alloc(Transform.class);

		// phase 3: down
		float additionalDownStep = (wasOnGround /*&& !onGround()*/) ? stepHeight : 0.0f;
		Vector3f step_drop = Stack.alloc(Vector3f.class);
		step_drop.scale(currentStepOffset + additionalDownStep, upAxisDirection[upAxis]);
		float downVelocity = (additionalDownStep == 0.0f && verticalVelocity < 0.0f ? -verticalVelocity : 0.0f) * dt;
		Vector3f gravity_drop = Stack.alloc(Vector3f.class);
		gravity_drop.scale(downVelocity, upAxisDirection[upAxis]);
		targetPosition.sub(step_drop);
		targetPosition.sub(gravity_drop);

		start.setIdentity();
		end.setIdentity();
		start.origin.set(currentPosition);
		end.origin.set(targetPosition);

		//We  do an 1k ray and if there is no floor anywhere below us, 
		//just disable gravity because infinite falling is NEVER useful, Note I borrow start and set it back
		start.origin.y -= 1000;
		KinematicClosestNotMeRayResultCallback cb = new KinematicClosestNotMeRayResultCallback(ghostObject);
		cb.collisionFilterGroup = ghostObject.getBroadphaseHandle().collisionFilterGroup;
		cb.collisionFilterMask = ghostObject.getBroadphaseHandle().collisionFilterMask;
		ghostObject.rayTest(currentPosition, start.origin, cb);
		start.origin.y += 1000;
		if (!cb.hasHit())
		{
			// take stepHeight back off, it was added in during step up
			currentPosition.y -= currentStepOffset;
			verticalVelocity = 0.0f;
			verticalOffset = 0.0f;
			return;
		}

		KinematicClosestNotMeConvexResultCallback callback = new KinematicClosestNotMeConvexResultCallback(ghostObject,
				upAxisDirection[upAxis], maxSlopeCosine);
		callback.collisionFilterGroup = ghostObject.getBroadphaseHandle().collisionFilterGroup;
		callback.collisionFilterMask = ghostObject.getBroadphaseHandle().collisionFilterMask;

		if (useGhostObjectSweepTest)
		{
			ghostObject.convexSweepTest(convexShape, start, end, callback, collisionWorld.getDispatchInfo().allowedCcdPenetration);
		}
		else
		{
			// this is a dodgy call that fails on GImapcts see the ghostObject.convexSweepTest above for a better option
			collisionWorld.convexSweepTest(convexShape, start, end, callback);
		}

		if (callback.hasHit())
		{
			// we dropped a fraction of the height -> hit floor
			currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
			verticalVelocity = 0.0f;
			verticalOffset = 0.0f;
		}
		else
		{

			// we dropped the full height
			currentPosition.set(targetPosition);
		}

	}

	public void setFreeFly(boolean ff)
	{
		isFreeFly = ff;
	}

	////////////////////////////////////////////////////////////////////////////

	private static class KinematicClosestNotMeRayResultCallback extends CollisionWorld.ClosestRayResultCallback
	{
		protected CollisionObject me;

		public KinematicClosestNotMeRayResultCallback(CollisionObject me)
		{
			super(new Vector3f(), new Vector3f());
			this.me = me;
		}

		@Override
		public float addSingleResult(CollisionWorld.LocalRayResult rayResult, boolean normalInWorldSpace)
		{
			if (rayResult.collisionObject == me)
			{
				return 1.0f;
			}

			return super.addSingleResult(rayResult, normalInWorldSpace);
		}
	}

	////////////////////////////////////////////////////////////////////////////

	private static class KinematicClosestNotMeConvexResultCallback extends CollisionWorld.ClosestConvexResultCallback
	{
		protected CollisionObject me;

		protected final Vector3f up;

		protected float minSlopeDot;

		public KinematicClosestNotMeConvexResultCallback(CollisionObject me, final Vector3f up, float minSlopeDot)
		{
			super(new Vector3f(), new Vector3f());
			this.me = me;
			this.up = up;
			this.minSlopeDot = minSlopeDot;
		}

		@Override
		public float addSingleResult(CollisionWorld.LocalConvexResult convexResult, boolean normalInWorldSpace)
		{
			if (convexResult.hitCollisionObject == me)
			{
				return 1.0f;
			}

			Vector3f hitNormalWorld2;
			if (normalInWorldSpace)
			{
				hitNormalWorld2 = convexResult.hitNormalLocal;
			}
			else
			{
				//need to transform normal into worldspace
				hitNormalWorld2 = Stack.alloc(Vector3f.class);
				hitCollisionObject = convexResult.hitCollisionObject;
				hitCollisionObject.getWorldTransform(Stack.alloc(Transform.class)).basis.transform(convexResult.hitNormalLocal,
						hitNormalWorld2);
			}

			float dotUp = up.dot(hitNormalWorld2);
			if (dotUp < minSlopeDot)
			{
				return 1.0f;
			}

			return super.addSingleResult(convexResult, normalInWorldSpace);
		}
	}

	public static interface CharacterPositionListener
	{
		public void positionChanged(Vector3f newPosition, Quat4f newRotation);
	}

	/**
	 * Returns the reflection direction of a ray going 'direction' hitting a surface
	 * with normal 'normal'.<p>
	 *
	 * From: http://www-cs-students.stanford.edu/~adityagp/final/node3.html
	 */
	protected static Vector3f computeReflectionDirection(Vector3f direction, Vector3f normal, Vector3f out)
	{
		// return direction - (btScalar(2.0) * direction.dot(normal)) * normal;
		out.set(normal);
		out.scale(-2.0f * direction.dot(normal));
		out.add(direction);

		return out;
	}

	/**
	 * Returns the portion of 'direction' that is parallel to 'normal'
	 */
	protected static Vector3f parallelComponent(Vector3f direction, Vector3f normal, Vector3f out)
	{

		//btScalar magnitude = direction.dot(normal);
		//return normal * magnitude;
		out.set(normal);
		out.scale(direction.dot(normal));

		return out;
	}

	/**
	 * Returns the portion of 'direction' that is perpindicular to 'normal'
	 */
	protected static Vector3f perpindicularComponent(Vector3f direction, Vector3f normal, Vector3f out)
	{

		//return direction - parallelComponent(direction, normal);
		Vector3f perpendicular = parallelComponent(direction, normal, out);
		perpendicular.scale(-1);
		perpendicular.add(direction);

		return perpendicular;
	}

}
