#ifndef _EVICTING_QUEUE_H
#define _EVICTING_QUEUE_H

class EvictingQueue{
	private:
		int startIndex;
		int endIndex;
		float *data;
		int maxSize;
	public:
		void empty();
		void enqueue(float value);
		float getMostRecentItem(int index);
		float sum();
		float average();
		int length();
		EvictingQueue(int size);
};

#endif
