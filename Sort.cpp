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
		quickSort(a, l, i - 1); /* 递归调用 */
		quickSort(a, i + 1, r); /* 递归调用 */
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
	int c = start;            // 当前(current)节点的位置
	int l = 2 * c + 1;        // 左(left)孩子的位置
	int tmp = a[c];            // 当前(current)节点的大小
	for (; l <= end; c = l, l = 2 * l + 1)
	{
		// "l"是左孩子，"l+1"是右孩子
		if (l < end && a[l] < a[l + 1])
			l++;        // 左右两孩子中选择较大者，即m_heap[l+1]
		if (tmp >= a[l])
			break;        // 调整结束
		else            // 交换值
		{
			a[c] = a[l];
			a[l] = tmp;
		}
	}
	return 0;
}

/*
  43  * 堆排序(从小到大)
  44  *
  45  * 参数说明：
  46  *     a -- 待排序的数组
  47  *     n -- 数组的长度
  48  */
int Sort::heapSortArc(int* a, int n)
{
	int i, tmp;

	// 从(n/2-1) --> 0逐次遍历。遍历之后，得到的数组实际上是一个(最大)二叉堆。
	for (i = n / 2 - 1; i >= 0; i--)
		maxHeapDown(a, i, n - 1);

	// 从最后一个元素开始对序列进行调整，不断的缩小调整的范围直到第一个元素
	for (i = n - 1; i > 0; i--)
	{
		// 交换a[0]和a[i]。交换后，a[i]是a[0...i]中最大的。
		tmp = a[0];
		a[0] = a[i];
		a[i] = tmp;
		// 调整a[0...i-1]，使得a[0...i-1]仍然是一个最大堆。
			// 即，保证a[i-1]是a[0...i-1]中的最大值。
		maxHeapDown(a, 0, i - 1);
	}
	return 0;
}
void Sort::merge(int a[], int start, int mid, int end)
{
	int *tmp = (int *)malloc((end - start + 1) * sizeof(int));    // tmp是汇总2个有序区的临时区域
	int i = start;            // 第1个有序区的索引
	int j = mid + 1;        // 第2个有序区的索引
	int k = 0;                // 临时区域的索引

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

	// 将排序后的元素，全部都整合到数组a中。
	for (i = 0; i < k; i++)
		a[start + i] = tmp[i];

	free(tmp);
}

/*
* 归并排序(从上往下)
*
* 参数说明：
*     a -- 待排序的数组
*     start -- 数组的起始地址
*     endi -- 数组的结束地址
*/
void Sort::merge_sort_up2down(int a[], int start, int end)
{
	if (a == NULL || start >= end)
		return;

	int mid = (end + start) / 2;
	merge_sort_up2down(a, start, mid); // 递归排序a[start...mid]
	merge_sort_up2down(a, mid + 1, end); // 递归排序a[mid+1...end]

										 // a[start...mid] 和 a[mid...end]是两个有序空间，
										 // 将它们排序成一个有序空间a[start...end]
	merge(a, start, mid, end);
}