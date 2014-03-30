package com.bulletphysics.collision.dispatch;

import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.dispatch.CollisionWorld.ConvexResultCallback;
import com.bulletphysics.collision.dispatch.CollisionWorld.LocalConvexResult;
import com.bulletphysics.collision.dispatch.CollisionWorld.LocalShapeInfo;
import com.bulletphysics.collision.narrowphase.ConvexCast;
import com.bulletphysics.collision.narrowphase.ConvexCast.CastResult;
import com.bulletphysics.collision.narrowphase.GjkConvexCast;
import com.bulletphysics.collision.narrowphase.TriangleConvexcastCallback;
import com.bulletphysics.collision.narrowphase.VoronoiSimplexSolver;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.TransformUtil;
import com.bulletphysics.linearmath.VectorUtil;

import cz.advel.stack.Stack;

public class LocalPairCachingGhostObject extends PairCachingGhostObject
{

	//TODO: merge with parent
	/**
	 * This method is an exact copy of GhostObject.convexSweepTest
	 * With one call changed due to a bug
	 * original bullet not correctly copied for jbullet 
	 * see http://projects.developer.nokia.com/gles2phys/browser/trunk/src/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp?rev=12
	 * @see com.bulletphysics.collision.dispatch.GhostObject#convexSweepTest(com.bulletphysics.collision.shapes.ConvexShape, com.bulletphysics.linearmath.Transform, com.bulletphysics.linearmath.Transform, com.bulletphysics.collision.dispatch.CollisionWorld.ConvexResultCallback, float)
	 */
	@Override
	public void convexSweepTest(ConvexShape castShape, Transform convexFromWorld, Transform convexToWorld,
			CollisionWorld.ConvexResultCallback resultCallback, float allowedCcdPenetration)
	{
		Transform convexFromTrans = Stack.alloc(Transform.class);
		Transform convexToTrans = Stack.alloc(Transform.class);

		convexFromTrans.set(convexFromWorld);
		convexToTrans.set(convexToWorld);

		Vector3f castShapeAabbMin = Stack.alloc(Vector3f.class);
		Vector3f castShapeAabbMax = Stack.alloc(Vector3f.class);

		// compute AABB that encompasses angular movement
		{
			Vector3f linVel = new Vector3f();
			Vector3f angVel = new Vector3f();
			TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1f, linVel, angVel);
			Transform R = Stack.alloc(Transform.class);
			R.setIdentity();
			R.setRotation(convexFromTrans.getRotation(Stack.alloc(Quat4f.class)));
			castShape.calculateTemporalAabb(R, linVel, angVel, 1f, castShapeAabbMin, castShapeAabbMax);
		}

		Transform tmpTrans = Stack.alloc(Transform.class);

		// go over all objects, and if the ray intersects their aabb + cast shape aabb,
		// do a ray-shape query using convexCaster (CCD)
		for (int i = 0; i < overlappingObjects.size(); i++)
		{
			CollisionObject collisionObject = overlappingObjects.getQuick(i);

			// only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject.getBroadphaseHandle()))
			{
				//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
				Vector3f collisionObjectAabbMin = Stack.alloc(Vector3f.class);
				Vector3f collisionObjectAabbMax = Stack.alloc(Vector3f.class);
				collisionObject.getCollisionShape().getAabb(collisionObject.getWorldTransform(tmpTrans), collisionObjectAabbMin,
						collisionObjectAabbMax);
				AabbUtil2.aabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
				float[] hitLambda = new float[]
				{ 1f }; // could use resultCallback.closestHitFraction, but needs testing
				Vector3f hitNormal = new Vector3f();
				if (AabbUtil2.rayAabb(convexFromWorld.origin, convexToWorld.origin, collisionObjectAabbMin, collisionObjectAabbMax,
						hitLambda, hitNormal))
				{
					//CHANGE CHANGE from CollisionWorld.objectQuerySingle
					objectQuerySingle(castShape, convexFromTrans, convexToTrans, collisionObject, collisionObject.getCollisionShape(),
							collisionObject.getWorldTransform(tmpTrans), resultCallback, allowedCcdPenetration);
				}
			}
		}
	}

	/**This is an exact copy of CollisionWorld.objectQuerySingle with one line changed (corrected to bullet original)
	 * and due to visibility issues the BridgeTriangleConvexcastCallback had to be copied below
	 * objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
	 */
	public static void objectQuerySingle(ConvexShape castShape, Transform convexFromTrans, Transform convexToTrans,
			CollisionObject collisionObject, CollisionShape collisionShape, Transform colObjWorldTransform,
			ConvexResultCallback resultCallback, float allowedPenetration)
	{
		if (collisionShape.isConvex())
		{
			CastResult castResult = new CastResult();
			castResult.allowedPenetration = allowedPenetration;
			castResult.fraction = 1f; // ??

			ConvexShape convexShape = (ConvexShape) collisionShape;
			VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();
			//GjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new GjkEpaPenetrationDepthSolver();

			// JAVA TODO: should be convexCaster1
			//ContinuousConvexCollision convexCaster1(castShape,convexShape,&simplexSolver,&gjkEpaPenetrationSolver);
			GjkConvexCast convexCaster2 = new GjkConvexCast(castShape, convexShape, simplexSolver);
			//btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

			ConvexCast castPtr = convexCaster2;

			if (castPtr.calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform, castResult))
			{
				// add hit
				if (castResult.normal.lengthSquared() > 0.0001f)
				{
					if (castResult.fraction < resultCallback.closestHitFraction)
					{
						castResult.normal.normalize();
						LocalConvexResult localConvexResult = new LocalConvexResult(collisionObject, null, castResult.normal,
								castResult.hitPoint, castResult.fraction);

						boolean normalInWorldSpace = true;
						resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
					}
				}
			}
		}
		else
		{
			if (collisionShape.isConcave())
			{
				if (collisionShape.getShapeType() == BroadphaseNativeType.TRIANGLE_MESH_SHAPE_PROXYTYPE)
				{
					BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape) collisionShape;
					Transform worldTocollisionObject = Stack.alloc(Transform.class);
					worldTocollisionObject.inverse(colObjWorldTransform);

					Vector3f convexFromLocal = Stack.alloc(Vector3f.class);
					convexFromLocal.set(convexFromTrans.origin);
					worldTocollisionObject.transform(convexFromLocal);

					Vector3f convexToLocal = Stack.alloc(Vector3f.class);
					convexToLocal.set(convexToTrans.origin);
					worldTocollisionObject.transform(convexToLocal);

					// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
					Transform rotationXform = Stack.alloc(Transform.class);
					Matrix3f tmpMat = Stack.alloc(Matrix3f.class);
					tmpMat.mul(worldTocollisionObject.basis, convexToTrans.basis);
					rotationXform.set(tmpMat);

					BridgeTriangleConvexcastCallback tccb = new BridgeTriangleConvexcastCallback(castShape, convexFromTrans, convexToTrans,
							resultCallback, collisionObject, triangleMesh, colObjWorldTransform);
					tccb.hitFraction = resultCallback.closestHitFraction;
					tccb.normalInWorldSpace = true;

					Vector3f boxMinLocal = Stack.alloc(Vector3f.class);
					Vector3f boxMaxLocal = Stack.alloc(Vector3f.class);
					castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);
					triangleMesh.performConvexcast(tccb, convexFromLocal, convexToLocal, boxMinLocal, boxMaxLocal);
				}
				else
				{
					//BvhTriangleMeshShape triangleMesh = (BvhTriangleMeshShape) collisionShape;
					// CHANGE CHANGE
					ConcaveShape triangleMesh = (ConcaveShape) collisionShape;
					Transform worldTocollisionObject = Stack.alloc(Transform.class);
					worldTocollisionObject.inverse(colObjWorldTransform);

					Vector3f convexFromLocal = Stack.alloc(Vector3f.class);
					convexFromLocal.set(convexFromTrans.origin);
					worldTocollisionObject.transform(convexFromLocal);

					Vector3f convexToLocal = Stack.alloc(Vector3f.class);
					convexToLocal.set(convexToTrans.origin);
					worldTocollisionObject.transform(convexToLocal);

					// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
					Transform rotationXform = Stack.alloc(Transform.class);
					Matrix3f tmpMat = Stack.alloc(Matrix3f.class);
					tmpMat.mul(worldTocollisionObject.basis, convexToTrans.basis);
					rotationXform.set(tmpMat);

					BridgeTriangleConvexcastCallback tccb = new BridgeTriangleConvexcastCallback(castShape, convexFromTrans, convexToTrans,
							resultCallback, collisionObject, triangleMesh, colObjWorldTransform);
					tccb.hitFraction = resultCallback.closestHitFraction;
					tccb.normalInWorldSpace = false;
					Vector3f boxMinLocal = Stack.alloc(Vector3f.class);
					Vector3f boxMaxLocal = Stack.alloc(Vector3f.class);
					castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);

					Vector3f rayAabbMinLocal = new Vector3f(convexFromLocal);
					VectorUtil.setMin(rayAabbMinLocal, convexToLocal);
					Vector3f rayAabbMaxLocal = new Vector3f(convexFromLocal);
					VectorUtil.setMax(rayAabbMaxLocal, convexToLocal);
					rayAabbMinLocal.add(boxMinLocal);
					rayAabbMaxLocal.add(boxMaxLocal);
					triangleMesh.processAllTriangles(tccb, rayAabbMinLocal, rayAabbMaxLocal);
				}
			}
			else
			{
				// todo: use AABB tree or other BVH acceleration structure!
				if (collisionShape.isCompound())
				{
					CompoundShape compoundShape = (CompoundShape) collisionShape;
					for (int i = 0; i < compoundShape.getNumChildShapes(); i++)
					{
						Transform childTrans = compoundShape.getChildTransform(i, Stack.alloc(Transform.class));
						CollisionShape childCollisionShape = compoundShape.getChildShape(i);
						Transform childWorldTrans = Stack.alloc(Transform.class);
						childWorldTrans.mul(colObjWorldTransform, childTrans);
						// replace collision shape so that callback can determine the triangle
						CollisionShape saveCollisionShape = collisionObject.getCollisionShape();
						collisionObject.internalSetTemporaryCollisionShape(childCollisionShape);
						objectQuerySingle(castShape, convexFromTrans, convexToTrans, collisionObject, childCollisionShape, childWorldTrans,
								resultCallback, allowedPenetration);
						// restore
						collisionObject.internalSetTemporaryCollisionShape(saveCollisionShape);
					}
				}
			}
		}
	}

	/**
	 * Copy from CollisionWorld due to visibility issues, With triangleMesh set correctly from bullet
	 * @author philip
	 *
	 */
	private static class BridgeTriangleConvexcastCallback extends TriangleConvexcastCallback
	{
		public ConvexResultCallback resultCallback;

		public CollisionObject collisionObject;

		public ConcaveShape triangleMesh;

		public boolean normalInWorldSpace;

		public BridgeTriangleConvexcastCallback(ConvexShape castShape, Transform from, Transform to, ConvexResultCallback resultCallback,
				CollisionObject collisionObject, ConcaveShape triangleMesh, Transform triangleToWorld)
		{
			super(castShape, from, to, triangleToWorld, triangleMesh.getMargin());
			this.resultCallback = resultCallback;
			this.collisionObject = collisionObject;
			this.triangleMesh = triangleMesh;
		}

		@Override
		public float reportHit(Vector3f hitNormalLocal, Vector3f hitPointLocal, float hitFraction, int partId, int triangleIndex)
		{
			LocalShapeInfo shapeInfo = new LocalShapeInfo();
			shapeInfo.shapePart = partId;
			shapeInfo.triangleIndex = triangleIndex;
			if (hitFraction <= resultCallback.closestHitFraction)
			{
				LocalConvexResult convexResult = new LocalConvexResult(collisionObject, shapeInfo, hitNormalLocal, hitPointLocal,
						hitFraction);
				return resultCallback.addSingleResult(convexResult, normalInWorldSpace);
			}
			return hitFraction;
		}
	}
}
