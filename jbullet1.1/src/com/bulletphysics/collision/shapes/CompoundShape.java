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

package com.bulletphysics.collision.shapes;

import org.jogamp.vecmath.Matrix3f;
import org.jogamp.vecmath.Vector3f;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectArrayList;

import cz.advel.stack.Stack;

// JAVA NOTE: CompoundShape from 2.71

/**
 * CompoundShape allows to store multiple other {@link CollisionShape}s. This allows
 * for moving concave collision objects. This is more general than the {@link BvhTriangleMeshShape}.
 * 
 * @author jezek2
 */
public class CompoundShape extends CollisionShape
{
	private final ObjectArrayList<CompoundShapeChild> children = new ObjectArrayList<CompoundShapeChild>();

	private final Vector3f localAabbMin = new Vector3f(1e30f, 1e30f, 1e30f);

	private final Vector3f localAabbMax = new Vector3f(-1e30f, -1e30f, -1e30f);

	private Vector3f localHalfExtents = new Vector3f();

	private Vector3f localCenter = new Vector3f();

	private OptimizedBvh aabbTree = null;

	private float collisionMargin = 0f;

	public void addChildShape(Transform localTransform, CollisionShape shape)
	{
		//m_childTransforms.push_back(localTransform);
		//m_childShapes.push_back(shape);
		CompoundShapeChild child = new CompoundShapeChild();
		child.transform.set(localTransform);
		child.childShape = shape;
		child.childShapeType = shape.getShapeType();
		child.childMargin = shape.getMargin();

		children.add(child);

		// extend the local aabbMin/aabbMax
		Vector3f _localAabbMin = Stack.alloc(Vector3f.class), _localAabbMax = Stack.alloc(Vector3f.class);
		shape.getAabb(localTransform, _localAabbMin, _localAabbMax);

		// JAVA NOTE: rewritten
		//		for (int i=0;i<3;i++)
		//		{
		//			if (this.localAabbMin[i] > _localAabbMin[i])
		//			{
		//				this.localAabbMin[i] = _localAabbMin[i];
		//			}
		//			if (this.localAabbMax[i] < _localAabbMax[i])
		//			{
		//				this.localAabbMax[i] = _localAabbMax[i];
		//			}
		//		}
		VectorUtil.setMin(this.localAabbMin, _localAabbMin);
		VectorUtil.setMax(this.localAabbMax, _localAabbMax);
		updateCenterAndExtents();
	}

	public void updateChildTransform(int childIndex, Transform newChildTransform)
	{
		children.getQuick(childIndex).transform.set(newChildTransform);

		///update the dynamic aabb tree
		//Vector3f _localAabbMin = Stack.alloc(Vector3f.class), _localAabbMax = Stack.alloc(Vector3f.class);
		//children.getQuick(childIndex).childShape.getAabb(newChildTransform, _localAabbMin, _localAabbMax);

		//VectorUtil.setMin(this.localAabbMin, _localAabbMin);
		//VectorUtil.setMax(this.localAabbMax, _localAabbMax);

		recalculateLocalAabb();
	}

	/**
	 * Remove all children shapes that contain the specified shape.
	 */
	public void removeChildShape(CollisionShape shape)
	{
		boolean done_removing;

		// Find the children containing the shape specified, and remove those children.
		do
		{
			done_removing = true;

			for (int i = 0; i < children.size(); i++)
			{
				if (children.getQuick(i).childShape == shape)
				{
					children.removeQuick(i);
					done_removing = false; // Do another iteration pass after removing from the vector
					break;
				}
			}
		}
		while (!done_removing);

		recalculateLocalAabb();
	}

	public int getNumChildShapes()
	{
		return children.size();
	}

	public int getChildShapeIndex(CollisionShape child)
	{
		for (int i = 0; i < children.size(); i++)
		{
			if (children.getQuick(i).childShape == child)
			{
				return i;
			}
		}
		return -1;
	}

	public CollisionShape getChildShape(int index)
	{
		return children.getQuick(index).childShape;
	}

	/**
	 * DO NOT alter this, use updateChildTransform()
	 * @param index
	 * @param out
	 * @return
	 */
	public Transform getChildTransform(int index, Transform out)
	{
		out.set(children.getQuick(index).transform);
		return out;
	}

	public ObjectArrayList<CompoundShapeChild> getChildList()
	{
		return children;
	}

	//TODO: why is local scaling ignored, I suspect it should not be used for compound shapes??
	private void updateCenterAndExtents()
	{
		localHalfExtents.sub(localAabbMax, localAabbMin);
		localHalfExtents.x = (localHalfExtents.x * 0.5f) + collisionMargin;
		localHalfExtents.y = (localHalfExtents.y * 0.5f) + collisionMargin;
		localHalfExtents.z = (localHalfExtents.z * 0.5f) + collisionMargin;

		localCenter.add(localAabbMax, localAabbMin);
		localCenter.x *= 0.5f;
		localCenter.y *= 0.5f;
		localCenter.z *= 0.5f;
	}

	//deburners
	private Matrix3f abs_b = new Matrix3f();
	private Vector3f center = new Vector3f();
	private Vector3f extent = new Vector3f();

	/**
	 * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
	 */
	@Override
	public void getAabb(Transform trans, Vector3f aabbMin, Vector3f aabbMax)
	{
		synchronized (abs_b)
		{
			abs_b.set(trans.basis);
			MatrixUtil.absolute(abs_b);

			center.set(localCenter);
			trans.transform(center);

			// massively unrolled 
			extent.x = (abs_b.m00 * localHalfExtents.x + abs_b.m01 * localHalfExtents.y + abs_b.m02 * localHalfExtents.z);
			extent.y = (abs_b.m10 * localHalfExtents.x + abs_b.m11 * localHalfExtents.y + abs_b.m12 * localHalfExtents.z);
			extent.z = (abs_b.m20 * localHalfExtents.x + abs_b.m21 * localHalfExtents.y + abs_b.m22 * localHalfExtents.z);

			aabbMin.sub(center, extent);
			aabbMax.add(center, extent);
		}

	}

	/**
	 * Re-calculate the local Aabb. Is called at the end of removeChildShapes.
	 * Use this yourself if you modify the children or their transforms.
	 */
	public void recalculateLocalAabb()
	{

		// Recalculate the local aabb
		// Brute force, it iterates over all the shapes left.
		localAabbMin.set(1e30f, 1e30f, 1e30f);
		localAabbMax.set(-1e30f, -1e30f, -1e30f);

		Vector3f tmpLocalAabbMin = Stack.alloc(Vector3f.class);
		Vector3f tmpLocalAabbMax = Stack.alloc(Vector3f.class);

		// extend the local aabbMin/aabbMax
		for (int j = 0; j < children.size(); j++)
		{
			children.getQuick(j).childShape.getAabb(children.getQuick(j).transform, tmpLocalAabbMin, tmpLocalAabbMax);

			/*	replaced with cleaner util
			 for (int i = 0; i < 3; i++)
				{
					if (VectorUtil.getCoord(localAabbMin, i) > VectorUtil.getCoord(tmpLocalAabbMin, i))
					{
						VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmpLocalAabbMin, i));
					}
					if (VectorUtil.getCoord(localAabbMax, i) < VectorUtil.getCoord(tmpLocalAabbMax, i))
					{
						VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmpLocalAabbMax, i));
					}
				}*/
			VectorUtil.setMin(this.localAabbMin, tmpLocalAabbMin);
			VectorUtil.setMax(this.localAabbMax, tmpLocalAabbMax);
		}
		updateCenterAndExtents();
	}

	/**
	 * Careful!! this appears to be unusable!
	 */
	@Override
	public void setLocalScaling(Vector3f scaling)
	{
		throw new UnsupportedOperationException();
	}

	@Override
	public Vector3f getLocalScaling(Vector3f out)
	{
		throw new UnsupportedOperationException();
	}

	@Override
	public void calculateLocalInertia(float mass, Vector3f inertia)
	{
		// approximation: take the inertia from the aabb for now
		Transform ident = Stack.alloc(Transform.class);
		ident.setIdentity();
		Vector3f aabbMin = Stack.alloc(Vector3f.class), aabbMax = Stack.alloc(Vector3f.class);
		getAabb(ident, aabbMin, aabbMax);

		Vector3f halfExtents = Stack.alloc(Vector3f.class);
		halfExtents.sub(aabbMax, aabbMin);
		halfExtents.scale(0.5f);

		float lx = 2f * halfExtents.x;
		float ly = 2f * halfExtents.y;
		float lz = 2f * halfExtents.z;

		inertia.x = (mass / 12f) * (ly * ly + lz * lz);
		inertia.y = (mass / 12f) * (lx * lx + lz * lz);
		inertia.z = (mass / 12f) * (lx * lx + ly * ly);
	}

	@Override
	public BroadphaseNativeType getShapeType()
	{
		return BroadphaseNativeType.COMPOUND_SHAPE_PROXYTYPE;
	}

	@Override
	public void setMargin(float margin)
	{
		collisionMargin = margin;
		updateCenterAndExtents();
	}

	@Override
	public float getMargin()
	{
		return collisionMargin;
	}

	@Override
	public String getName()
	{
		return "Compound";
	}

	// this is optional, but should make collision queries faster, by culling non-overlapping nodes
	// void	createAabbTreeFromChildren();

	public OptimizedBvh getAabbTree()
	{
		return aabbTree;
	}

	/**
	 * Computes the exact moment of inertia and the transform from the coordinate
	 * system defined by the principal axes of the moment of inertia and the center
	 * of mass to the current coordinate system. "masses" points to an array
	 * of masses of the children. The resulting transform "principal" has to be
	 * applied inversely to all children transforms in order for the local coordinate
	 * system of the compound shape to be centered at the center of mass and to coincide
	 * with the principal axes. This also necessitates a correction of the world transform
	 * of the collision object by the principal transform.
	 */
	public void calculatePrincipalAxisTransform(float[] masses, Transform principal, Vector3f inertia)
	{
		int n = children.size();

		float totalMass = 0;
		Vector3f center = Stack.alloc(Vector3f.class);
		center.set(0, 0, 0);
		for (int k = 0; k < n; k++)
		{
			center.scaleAdd(masses[k], children.getQuick(k).transform.origin, center);
			totalMass += masses[k];
		}
		center.scale(1f / totalMass);
		principal.origin.set(center);

		Matrix3f tensor = Stack.alloc(Matrix3f.class);
		tensor.setZero();

		for (int k = 0; k < n; k++)
		{
			Vector3f i = Stack.alloc(Vector3f.class);
			children.getQuick(k).childShape.calculateLocalInertia(masses[k], i);

			Transform t = children.getQuick(k).transform;
			Vector3f o = Stack.alloc(Vector3f.class);
			o.sub(t.origin, center);

			// compute inertia tensor in coordinate system of compound shape
			Matrix3f j = Stack.alloc(Matrix3f.class);
			j.transpose(t.basis);

			j.m00 *= i.x;
			j.m01 *= i.x;
			j.m02 *= i.x;
			j.m10 *= i.y;
			j.m11 *= i.y;
			j.m12 *= i.y;
			j.m20 *= i.z;
			j.m21 *= i.z;
			j.m22 *= i.z;

			j.mul(t.basis, j);

			// add inertia tensor
			tensor.add(j);

			// compute inertia tensor of pointmass at o
			float o2 = o.lengthSquared();
			j.setRow(0, o2, 0, 0);
			j.setRow(1, 0, o2, 0);
			j.setRow(2, 0, 0, o2);
			j.m00 += o.x * -o.x;
			j.m01 += o.y * -o.x;
			j.m02 += o.z * -o.x;
			j.m10 += o.x * -o.y;
			j.m11 += o.y * -o.y;
			j.m12 += o.z * -o.y;
			j.m20 += o.x * -o.z;
			j.m21 += o.y * -o.z;
			j.m22 += o.z * -o.z;

			// add inertia tensor of pointmass
			tensor.m00 += masses[k] * j.m00;
			tensor.m01 += masses[k] * j.m01;
			tensor.m02 += masses[k] * j.m02;
			tensor.m10 += masses[k] * j.m10;
			tensor.m11 += masses[k] * j.m11;
			tensor.m12 += masses[k] * j.m12;
			tensor.m20 += masses[k] * j.m20;
			tensor.m21 += masses[k] * j.m21;
			tensor.m22 += masses[k] * j.m22;
		}

		MatrixUtil.diagonalize(tensor, principal.basis, 0.00001f, 20);

		inertia.set(tensor.m00, tensor.m11, tensor.m22);
	}

}
