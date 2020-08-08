#ifndef MEAN_BUFFER_H
#define MEAN_BUFFER_H

// storing last n arbitrary values (ring buffer) and calculates mean
class MeanBuffer
{
public:
	MeanBuffer(int n): _n(n), _mean(0), _valuesStored(0){_values = new float[_n];}
	MeanBuffer(): MeanBuffer(100){}// default 100 values
	~MeanBuffer(){delete[] _values;}
	void push(float v){_values[_valuesStored%_n] = v; _valuesStored++;}
	void calculateMean(){
		int max_it = _valuesStored;
		if(max_it > _n)
			max_it = _n;
		float sum = 0.f;
		for(int i = 0; i < max_it; i++){
			sum += _values[i];	
		}
		if(max_it == 0)
			_mean = 0;
		else
			_mean = sum/max_it;
	}
	void reset(){_mean = 0; _valuesStored = 0;}

	float getMean(){return _mean;}
	const float* getValues(){return _values;}
	int getCapacity(){return _n;}

private:
	float _mean;
	float * _values;// holding n values
	int _valuesStored;// total amount of numbers pushed
	int _n;
};


#endif
