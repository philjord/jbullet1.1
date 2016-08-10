/*
 * Port of btHeightfieldTerrainShape by Dominic Browne <dominic.browne@hotmail.co.uk>
 *
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

package com.bulletphysics.dom;

import javax.vecmath.Matrix3f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ScalarType;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;

import cz.advel.stack.Stack;

public class HeightfieldTerrainShape extends ConcaveShape
{

	public static final int XAXIS = 0;

	public static final int YAXIS = 1;

	public static final int ZAXIS = 2;

	protected Vector3f m_localAabbMin = new Vector3f();

	protected Vector3f m_localAabbMax = new Vector3f();

	protected Vector3f m_localOrigin = new Vector3f();

	private Vector3f localHalfExtents = new Vector3f();

	// /terrain data
	protected int m_heightStickWidth;

	protected int m_heightStickLength;

	protected float m_minHeight;

	protected float m_maxHeight;

	protected float m_width;

	protected float m_length;

	protected float m_heightScale;

	protected float[] m_heightfieldDataFloat;

	protected ScalarType m_heightDataType;

	protected boolean m_flipQuadEdges;

	protected boolean m_useDiamondSubdivision;

	protected int m_upAxis;

	protected Vector3f m_localScaling = new Vector3f();

	public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength, float[] heightfieldData, float heightScale, float minHeight,
			float maxHeight, int upAxis, boolean flipQuadEdges)
	{

		initialize(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis, ScalarType.FLOAT,
				flipQuadEdges);
	}

	private void initialize(int heightStickWidth, int heightStickLength, float[] heightfieldData, float heightScale, float minHeight,
			float maxHeight, int upAxis, ScalarType f, boolean flipQuadEdges)
	{
		m_heightStickWidth = heightStickWidth;
		m_heightStickLength = heightStickLength;
		m_minHeight = minHeight * heightScale;
		m_maxHeight = maxHeight * heightScale;
		m_width = (heightStickWidth - 1);
		m_length = (heightStickLength - 1);
		m_heightScale = heightScale;
		m_heightfieldDataFloat = heightfieldData;
		m_heightDataType = ScalarType.FLOAT;
		m_flipQuadEdges = flipQuadEdges;
		m_useDiamondSubdivision = false;
		m_upAxis = upAxis;
		m_localScaling.set(1.f, 1.f, 1.f);

		// determine min/max axis-aligned bounding box (aabb) values
		switch (m_upAxis)
		{
		case 0:
		{
			m_localAabbMin.set(m_minHeight, 0, 0);
			m_localAabbMax.set(m_maxHeight, m_width, m_length);
			break;
		}
		case 1:
		{
			m_localAabbMin.set(0, m_minHeight, 0);
			m_localAabbMax.set(m_width, m_maxHeight, m_length);
			break;
		}
		case 2:
		{
			m_localAabbMin.set(0, 0, m_minHeight);
			m_localAabbMax.set(m_width, m_length, m_maxHeight);
			break;
		}
		}

		// remember origin (defined as exact middle of aabb)
		// m_localOrigin = btScalar(0.5) * (m_localAabbMin + m_localAabbMax);

		m_localOrigin.set(m_localAabbMin);
		m_localOrigin.add(m_localAabbMax);
		m_localOrigin.x = m_localOrigin.x * 0.5f;
		m_localOrigin.y = m_localOrigin.y * 0.5f;
		m_localOrigin.z = m_localOrigin.z * 0.5f;
		updateExtents();
	}

	//deburners
	private Vector3f v0 = new Vector3f();
	private Vector3f v1 = new Vector3f();
	private Vector3f v2 = new Vector3f();

	private Vector3f localAabbMin = new Vector3f();
	private Vector3f localAabbMax = new Vector3f();
	private int[] quantizedAabbMin = new int[3];
	private int[] quantizedAabbMax = new int[3];
	private Vector3f[] vertices = new Vector3f[3];

	@Override
	public void processAllTriangles(TriangleCallback callback, Vector3f aabbMin, Vector3f aabbMax)
	{

		localAabbMin.x = aabbMin.x * (1.f / m_localScaling.x);
		localAabbMin.y = aabbMin.y * (1.f / m_localScaling.y);
		localAabbMin.z = aabbMin.z * (1.f / m_localScaling.z);

		localAabbMax.x = aabbMax.x * (1.f / m_localScaling.x);
		localAabbMax.y = aabbMax.y * (1.f / m_localScaling.y);
		localAabbMax.z = aabbMax.z * (1.f / m_localScaling.z);

		localAabbMin.add(m_localOrigin);
		localAabbMax.add(m_localOrigin);

		// quantize the aabbMin and aabbMax, and adjust the start/end ranges

		quantizeWithClamp(quantizedAabbMin, localAabbMin);
		quantizeWithClamp(quantizedAabbMax, localAabbMax);

		// expand the min/max quantized values
		// this is to catch the case where the input aabb falls between grid points!
		for (int i = 0; i < 3; ++i)
		{
			quantizedAabbMin[i]--;
			quantizedAabbMax[i]++;
		}

		int startX = 0;
		int endX = m_heightStickWidth - 1;
		int startJ = 0;
		int endJ = m_heightStickLength - 1;

		switch (m_upAxis)
		{
		case 0:
		{
			if (quantizedAabbMin[1] > startX)
				startX = quantizedAabbMin[1];
			if (quantizedAabbMax[1] < endX)
				endX = quantizedAabbMax[1];
			if (quantizedAabbMin[2] > startJ)
				startJ = quantizedAabbMin[2];
			if (quantizedAabbMax[2] < endJ)
				endJ = quantizedAabbMax[2];
			break;
		}
		case 1:
		{
			if (quantizedAabbMin[0] > startX)
				startX = quantizedAabbMin[0];
			if (quantizedAabbMax[0] < endX)
				endX = quantizedAabbMax[0];
			if (quantizedAabbMin[2] > startJ)
				startJ = quantizedAabbMin[2];
			if (quantizedAabbMax[2] < endJ)
				endJ = quantizedAabbMax[2];
			break;
		}

		case 2:
		{
			if (quantizedAabbMin[0] > startX)
				startX = quantizedAabbMin[0];
			if (quantizedAabbMax[0] < endX)
				endX = quantizedAabbMax[0];
			if (quantizedAabbMin[1] > startJ)
				startJ = quantizedAabbMin[1];
			if (quantizedAabbMax[1] < endJ)
				endJ = quantizedAabbMax[1];
			break;
		}
		}

		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;

		for (int j = startJ; j < endJ; j++)
		{
			for (int x = startX; x < endX; x++)
			{
				// Vector3f vertices[3];

				if (m_flipQuadEdges || (m_useDiamondSubdivision && (((j + x) & 1) != 0)))
				{
					// first triangle
					getVertex(x, j, vertices[0]);
					getVertex(x + 1, j, vertices[1]);
					getVertex(x + 1, j + 1, vertices[2]);
					callback.processTriangle(vertices, x, j);
					// callback->processTriangle(vertices,x,j);
					// second triangle
					getVertex(x, j, vertices[0]);
					getVertex(x + 1, j + 1, vertices[1]);
					getVertex(x, j + 1, vertices[2]);
					// callback->processTriangle(vertices,x,j);
					callback.processTriangle(vertices, x, j);
				}
				else
				{
					// first triangle
					getVertex(x, j, vertices[0]);
					getVertex(x, j + 1, vertices[1]);
					getVertex(x + 1, j, vertices[2]);
					// callback->processTriangle(vertices,x,j);
					callback.processTriangle(vertices, x, j);
					// second triangle
					getVertex(x + 1, j, vertices[0]);
					getVertex(x, j + 1, vertices[1]);
					getVertex(x + 1, j + 1, vertices[2]);
					// callback->processTriangle(vertices,x,j);
					callback.processTriangle(vertices, x, j);
				}
			}
		}

	}

	// / this returns the vertex in bullet-local coordinates
	private void getVertex(int x, int y, Vector3f vertex)
	{
		float height = getRawHeightFieldValue(x, y);

		switch (m_upAxis)
		{
		case 0:
		{
			vertex.set(height - m_localOrigin.x, (-m_width / 2.0f) + x, (-m_length / 2.0f) + y);
			break;
		}
		case 1:
		{
			vertex.set((-m_width / 2.0f) + x, height - m_localOrigin.y, (-m_length / 2.0f) + y);
			break;
		}

		case 2:
		{
			vertex.set((-m_width / 2.0f) + x, (-m_length / 2.0f) + y, height - m_localOrigin.z);
			break;
		}
		}

		vertex.x = vertex.x * m_localScaling.x;
		vertex.y = vertex.y * m_localScaling.y;
		vertex.z = vertex.z * m_localScaling.z;
	}

	@Override
	public void calculateLocalInertia(float arg0, Vector3f inertia)
	{
		inertia.set(0.f, 0.f, 0.f);
	}

	//TODO: compound shape using margin but not scale!!
	// margin is added in at the end of the end of getAABB
	private void updateExtents()
	{
		localHalfExtents.sub(m_localAabbMax, m_localAabbMin);
		localHalfExtents.x = localHalfExtents.x * m_localScaling.x * 0.5f;
		localHalfExtents.y = localHalfExtents.y * m_localScaling.y * 0.5f;
		localHalfExtents.z = localHalfExtents.z * m_localScaling.z * 0.5f;

	}

	//deburners	
	private Matrix3f abs_b = new Matrix3f();
	private Vector3f center = new Vector3f();
	private Vector3f extent = new Vector3f();

	@Override
	public void getAabb(Transform t, Vector3f aabbMin, Vector3f aabbMax)
	{
		synchronized (abs_b)
		{

			/*Vector3f localOrigin(0, 0, 0);
			localOrigin[m_upAxis] = (m_minHeight + m_maxHeight) * 0.5f;  
			localOrigin *= m_localScaling;*/

			abs_b.set(t.basis);
			MatrixUtil.absolute(abs_b);

			center.set(t.origin);

			// massively unrolled 
			extent.x = (abs_b.m00 * localHalfExtents.x + abs_b.m01 * localHalfExtents.y + abs_b.m02 * localHalfExtents.z) + collisionMargin;
			extent.y = (abs_b.m10 * localHalfExtents.x + abs_b.m11 * localHalfExtents.y + abs_b.m12 * localHalfExtents.z) + collisionMargin;
			extent.z = (abs_b.m20 * localHalfExtents.x + abs_b.m21 * localHalfExtents.y + abs_b.m22 * localHalfExtents.z) + collisionMargin;

			aabbMin.sub(center, extent);
			aabbMax.add(center, extent);
		}
	}

	@Override
	public Vector3f getLocalScaling(Vector3f v)
	{
		v.set(m_localScaling);
		return v;
	}

	@Override
	public String getName()
	{
		return "Terrain";
	}

	@Override
	public BroadphaseNativeType getShapeType()
	{
		return BroadphaseNativeType.TERRAIN_SHAPE_PROXYTYPE;
	}

	@Override
	public void setLocalScaling(Vector3f scaling)
	{
		m_localScaling.set(scaling);
		updateExtents();
	}

	// / This returns the "raw" (user's initial) height, not the actual height.
	// / The actual height needs to be adjusted to be relative to the center
	// / of the heightfield's AABB.

	private float getRawHeightFieldValue(int x, int y)
	{
		return m_heightfieldDataFloat[(y * m_heightStickWidth) + x] * m_heightScale;
	}

	private static int getQuantized(float v)
	{
		if (v < 0.0)
		{
			return (int) (v - 0.5);
		}
		return (int) (v + 0.5);
	}

	// / given input vector, return quantized version
	/**
	 * This routine is basically determining the gridpoint indices for a given input vector, answering the question: "which gridpoint is closest to the provided point?".
	 *
	 * "with clamp" means that we restrict the point to be in the heightfield's axis-aligned bounding box.
	 */
	private void quantizeWithClamp(int[] out, Vector3f point)
	{
		Vector3f clampedPoint = Stack.alloc(point);
		VectorUtil.setMax(clampedPoint, m_localAabbMin);
		VectorUtil.setMin(clampedPoint, m_localAabbMax);

		out[0] = getQuantized(clampedPoint.x);
		out[1] = getQuantized(clampedPoint.y);
		out[2] = getQuantized(clampedPoint.z);
	}

	public int getHeightStickWidth()
	{
		return m_heightStickWidth;
	}

	public int getHeightStickLength()
	{
		return m_heightStickLength;
	}

	public float getMinHeight()
	{
		return m_minHeight;
	}

	public float getMaxHeight()
	{
		return m_maxHeight;
	}
}
