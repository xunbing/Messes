#include "stdafx.h"
#include "Sort.h"
int Sort::quickSort(int *a, int l, int r)
{
	if (l < r)
	{
		int i, j, x;
		i = l; j = r; x = a[i];
		while (i < j)
		{
			while (i<j&&a[j]>x)
				j--;
			if (i < j)
				a[i++] = a[j];
			while (i < j&&a[i] < x)
				i++;
			if (i < j)
				a[j--] = a[i];
		}
		a[i] = x;
		quickSort(a, l, i - 1); /* �ݹ���� */
		quickSort(a, i + 1, r); /* �ݹ���� */
	}


	return 0;
}
int Sort::insertSort(int *a, int n)
{
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < i; j++)
		{
			if (a[i] > a[j])
			{
				int tmp = a[j];
				a[j] = a[i];
				a[i] = tmp;
			}
		}
	}
	return 0;
}
int Sort::selectSort(int *a, int n)
{
	for (int i = 0; i < n; i++)
	{
		int Max = a[i];
		for (int j = i + 1; j < n; j++)
		{
			if (a[j] > Max)
			{
				a[i] = a[j];
				a[j] = Max;
				Max = a[i];
			}
		}
	}
	return 0;
}
int Sort::maxHeapDown(int* a, int start, int end)
{
	int c = start;            // ��ǰ(current)�ڵ��λ��
	int l = 2 * c + 1;        // ��(left)���ӵ�λ��
	int tmp = a[c];            // ��ǰ(current)�ڵ�Ĵ�С
	for (; l <= end; c = l, l = 2 * l + 1)
	{
		// "l"�����ӣ�"l+1"���Һ���
		if (l < end && a[l] < a[l + 1])
			l++;        // ������������ѡ��ϴ��ߣ���m_heap[l+1]
		if (tmp >= a[l])
			break;        // ��������
		else            // ����ֵ
		{
			a[c] = a[l];
			a[l] = tmp;
		}
	}
	return 0;
}

/*
  43  * ������(��С����)
  44  *
  45  * ����˵����
  46  *     a -- �����������
  47  *     n -- ����ĳ���
  48  */
int Sort::heapSortArc(int* a, int n)
{
	int i, tmp;

	// ��(n/2-1) --> 0��α���������֮�󣬵õ�������ʵ������һ��(���)����ѡ�
	for (i = n / 2 - 1; i >= 0; i--)
		maxHeapDown(a, i, n - 1);

	// �����һ��Ԫ�ؿ�ʼ�����н��е��������ϵ���С�����ķ�Χֱ����һ��Ԫ��
	for (i = n - 1; i > 0; i--)
	{
		// ����a[0]��a[i]��������a[i]��a[0...i]�����ġ�
		tmp = a[0];
		a[0] = a[i];
		a[i] = tmp;
		// ����a[0...i-1]��ʹ��a[0...i-1]��Ȼ��һ�����ѡ�
			// ������֤a[i-1]��a[0...i-1]�е����ֵ��
		maxHeapDown(a, 0, i - 1);
	}
	return 0;
}
void Sort::merge(int a[], int start, int mid, int end)
{
	int *tmp = (int *)malloc((end - start + 1) * sizeof(int));    // tmp�ǻ���2������������ʱ����
	int i = start;            // ��1��������������
	int j = mid + 1;        // ��2��������������
	int k = 0;                // ��ʱ���������

	while (i <= mid && j <= end)
	{
		if (a[i] <= a[j])
			tmp[k++] = a[i++];
		else
			tmp[k++] = a[j++];
	}

	while (i <= mid)
		tmp[k++] = a[i++];

	while (j <= end)
		tmp[k++] = a[j++];

	// ��������Ԫ�أ�ȫ�������ϵ�����a�С�
	for (i = 0; i < k; i++)
		a[start + i] = tmp[i];

	free(tmp);
}

/*
* �鲢����(��������)
*
* ����˵����
*     a -- �����������
*     start -- �������ʼ��ַ
*     endi -- ����Ľ�����ַ
*/
void Sort::merge_sort_up2down(int a[], int start, int end)
{
	if (a == NULL || start >= end)
		return;

	int mid = (end + start) / 2;
	merge_sort_up2down(a, start, mid); // �ݹ�����a[start...mid]
	merge_sort_up2down(a, mid + 1, end); // �ݹ�����a[mid+1...end]

										 // a[start...mid] �� a[mid...end]����������ռ䣬
										 // �����������һ������ռ�a[start...end]
	merge(a, start, mid, end);
}