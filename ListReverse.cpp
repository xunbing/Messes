// ListReverse.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include "Sort.h"
using namespace std;
struct Node
{
	int data;
	struct Node *link;
};

int reverse(Node *head)
{
	Node *p, *q, *r;
	p = head->link;
	q = p->link;
	p->link = NULL;
	while (q)
	{
		r = q->link;
	
		q->link = p;
		
		p = q;
		q = r;
	}
	head->link = p;
	return 0;
}
int main()
{
	int n;
	cin >> n;
	/*Node * head = new Node;
	head->link = NULL;
	for (int i = 0; i < n; i++)
	{
		int tmp;
		cin >> tmp;
		Node *newnode = new Node;
		newnode->data = tmp;
		newnode->link = head->link;
		head->link = newnode;
	}
	Node *p = head->link;
	while (p)
	{
		cout << p->data << " ";
		p = p->link;
	}
	reverse(head);
	p = head->link;
	while (p)
	{
		cout << p->data << " ";
		p = p->link;
	}*/
	int *a = new int[n];
	for (int i = 0; i < n; i++)
	{
		cin >> a[i];
	}
	Sort s;
	//s.quickSort(a,0,n-1);
	//s.insertSort(a, n);
	//s.selectSort(a, n);
	//s.heapSortArc(a, n);
	s.merge_sort_up2down(a, 0, n - 1);
	for (int i = 0; i < n; i++)
	{
		cout<<a[i]<<" ";
	}
    return 0;
}

