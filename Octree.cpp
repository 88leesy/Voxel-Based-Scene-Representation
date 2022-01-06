#pragma once

#include "Octree.h"

void BoundingBox::SetRadius(float radius)
{
	m_radius = radius;
}

BoundingBox* OctreeNode::GetBoundingBox()
{
	return m_pBoundBox;
}



void OctreeNode::AddChildNode(OctreeNode* node)
{
	// 이 노드에 자식 노드를 추가합니다.
	m_vChildren.push_back(node);
}

// 이 노드를 컬링합니다.
//void OctreeNode::CullNode(bool);          

 //자식 노드를 가져옵니다.
vector<OctreeNode*> const OctreeNode::GetChildNode(size_t)
{
	return m_vChildren;
}

//부모 노드를 설정합니다.
void OctreeNode::SetParent(OctreeNode* const node) 
{
	m_pParent = node;
}

//부모 노드를 반환합니다.
OctreeNode* const OctreeNode::GetParent()
{
	return m_pParent;
}


//vCenter : 노드의 중심 위치   fHalfWidth : 노드의 반지름     depthLimit : 현재 노드 진입 단계 
OctreeNode* OctreeNode::BuildOctree(Vector3 vCenter, float fHalfWidth, int depthLimit) {

	//제한된 진입단계에 도달하면 더 이상 자식 노드를 생성하지 않습니다.
	if (depthLimit < 0) {
		return NULL;
	}//if


	 //현재 노드를 생성
	OctreeNode* pOctNode = new OctreeNode();
	BoundingBox* pBBox = pOctNode->GetBoundingBox();
	pOctNode->SetPosition(vCenter);
	pBBox->SetRadius(fHalfWidth);

	//재귀적으로 8개의 자식 노드들을 생성합니다.
	Vector3 vOffset;
	Vector3 vChildCenter;
	float fStep = fHalfWidth * 0.5f;

	//8개의 자식 노드들에 대해서 중심 위치를 설정하고 트리를 생성.
	for (int iTree = 0; iTree < 8; ++iTree) {

		vOffset[0] = ((iTree & 1) ? fStep : -fStep);
		vOffset[1] = ((iTree & 2) ? fStep : -fStep);
		vOffset[2] = ((iTree & 4) ? fStep : -fStep);

		vChildCenter[0] = vOffset[0] + vCenter[0];
		vChildCenter[1] = vOffset[1] + vCenter[1];
		vChildCenter[2] = vOffset[2] + vCenter[2];

		pOctNode->AddChildNode(BuildOctree(vChildCenter, fStep, depthLimit - 1));

	}//for
	return pOctNode;

}//VOID BuildOctree( FLOAT fCenter, FLOAT fHalfWidth,  size_t depth )


//vObjPos : 판단하고자 하는 물체의 위치           // pNode : 판단하고자 하는 노드
bool  FindCurrentPosNode(const Vector3 vObjPos, OctreeNode* const pNode) {

	Vector3 pvNodePos = pNode->GetPosition();

	float fRadius = pNode->GetBoundingBox()->GetRadius();
	fRadius *= LOOSE_FACTOR;                                       // 느슨한 옥트리 같은 경우 반지름에 계수를 곱해줄 수 있습니다.

	float fMin, fMax;

	// 이 부분은 현재 물체가 이 노드 안에 완전히 포함되는 지 판단합니다.
	for (int index = 0; index < 3; ++index) {
		fMin = (pvNodePos)[index] - fRadius;
		fMax = (pvNodePos)[index] + fRadius;

		//포함되지 않으면 실패를 반환하죠.
		if (vObjPos[index] < fMin || vObjPos[index] > fMax) {
			return false;
		}//if
	} //for

	 //만약 노드 안에 물체가 완전히 속한다면 해당 노드를 반환  
	return true;


}//OctreeNode* FindCurrentPosNode( OctreeNode* const pNode )