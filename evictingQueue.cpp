#include "evictingQueue.h"
#include <stdio.h>

EvictingQueue::EvictingQueue(int size) {
	startIndex = 0;
	endIndex = 0;
	maxSize = size;
	data = new float[size];
}

void EvictingQueue::enqueue(float value) {
	startIndex = (startIndex + maxSize - 1) % maxSize;
	if (endIndex == startIndex) {
		endIndex = (endIndex + maxSize - 1) % maxSize;
	}
	data[startIndex] = value;
}

float EvictingQueue::getMostRecentItem(int index) {
	int actualIndex = (index + startIndex) % maxSize;
	return data[actualIndex];
}

int EvictingQueue::length() {
	if (endIndex >= startIndex) {
		return endIndex - startIndex;
	}
	return (maxSize - startIndex) + endIndex;
}

float EvictingQueue::sum() {
	float total = 0;
	int len = length();
	for(int i=0; i < len; i++) {
		total += getMostRecentItem(i);
	}
	return total;
}

float EvictingQueue::average() {
	return sum() / length();
}

void EvictingQueue::empty() {
	startIndex = 0;
	endIndex = 0;
}
