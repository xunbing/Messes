#pragma once
class Sort
{
private:
	int *a;
public:
	Sort()
	{
		
	}
	int quickSort(int *a,int l,int r);
	int insertSort(int *a, int n);
	int selectSort(int *a, int n);
	int maxHeapDown(int* a, int start, int end);
	int heapSortArc(int* a, int n);
	void merge(int a[], int start, int mid, int end);
	void merge_sort_up2down(int a[], int start, int end);
};