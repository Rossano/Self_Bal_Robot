/*
 * math_comp.h
 *
 *  Created on: 10 déc. 2015
 *      Author: rpantale
 */

#ifndef MATH_COMP_H_
#define MATH_COMP_H_

#ifdef ARDUINO
#include <Arduino.h>
#endif

#define N_ROW		4		// State matrix row number
#define N_COL		1		// State matrix column number

namespace math_comp {

	template<class T, size_t size> class _vector;
	template<class T, size_t size> _vector<T, size> operator+ (const _vector<T,size> &x, const _vector<T, size> &y);
	template<class T, size_t size> _vector<T, size> operator- (const _vector<T,size> &x, const _vector<T, size> &y);
	template<class T, size_t size> T operator* (const _vector<T,size> &x, const _vector<T, size> &y);
//	template<class T, size_t size> _vector<T, size> operator+ (const _vector<T,size> &x, const _vector<T, size> &y);
	template<class T, size_t size> _vector<T, size> operator* (const _vector<T,size> &x, const T &y);
//	template<class T, size_t size> _vector<T, size> operator= (const _vector<T,size> &x);
//	template<class T, size_t size> T operator[] (const size_t n);

	//
	//	Vector class
	//
	template <class T, size_t size>
	class _vector {
		//unsigned short dim;
		T el[size];		//	vector elements, by default it is by row
	public:
		_vector<T, size>();											// Default constructor
		_vector<T,size>(const T val[]);
		_vector<T, size>(const _vector<T, size> &);					// Constructor
		//_vector<T, size>(const T *val);
//
//		_vector<T> operator+(const _vector<T> &, const _vector<T> &);	// Vector sum
//		_vector<T> operator-(const _vector<T> &, const _vector<T> &);	// Vector difference
//		_vector<T> operator*(const _vector<T> &, const T);			// Vector scaling
//		T operator*(const _vector<T> &, const _vector<T> &);	// Vector by vector multiplication
		_vector<T, size> & operator= (const _vector<T,size> &x) ;
		T operator[](const size_t n) const;										// Get a vector element
//		T const & operator[](size_t n);
		T get_element(const size_t n) { return el[n]; }
		void set_element(const T val, const size_t n) { el[n] = val; }
//
		void printv();													// Print vector elements
		void setv(const _vector<T, size> &);											// Set vector elements
	};

	template <class T, size_t N, size_t M>
	class _matrix {
		//unsigned short row;
		//unsigned short col;
		T el[N][M];		// matrix elements
	public:
		_matrix();
		_matrix(const _matrix<T, N, M> &);
		void printm();
/*		_matrix<T> operator+(const _matrix<T> &, const _matrix<T> &);
		_matrix<T> operator-(const _matrix<T> &, const _matrix<T> &);
		_vector<T> operator*(const _matrix<T> &, const _vector<T> &);
		_matrix<T> operator*(const _matrix<T> &, const _matrix<T> &);
		_matrix<T> operator~(const _matrix<T> &);
		_vector<T> operator[](const int row);*/
	};

	template<class T, size_t size>
	//inline _vector<T,size>::template _vector<T,size>() {
	_vector<T,size>::_vector() {
		//dim = size;
		//el = new T[size];

		for(unsigned int i = 0; i < size; i++) el[i] = (T)0;
	}

	template<class T, size_t size>
	_vector<T,size>::_vector(const T val[]) {
		for(size_t i; i < size; i++) el[i] = val[i];
	}

	template<class T, size_t size>
	//inline _vector<T, size>::template _vector<T,size>(const _vector<T, size>& v) {
	_vector<T,size>::_vector(const _vector<T,size> &v) {
		//dim = size;
		//el = new T[size];

		for(size_t i = 0; i < size; i++) el[i] = v[i];
		//for (size_t i; i < size; i++) el[i] = v.get_element(i);

	}

	//template<class T, size_t size>
	//inline T _vector<T, size>::operator [](size_t n) {
	template<class T, size_t size> T _vector<T,size>::operator[] (const size_t n) const {
		return el[n];
	}
/*	template<class T, size_t size> T const & _vector<T,size>::operator[](size_t n) {
		return el[n];
	}*/

	template<class T, size_t size> _vector<T, size>& _vector<T,size>::operator= (const _vector<T,size> &x) {
		//_vector<T,size> res;

		for(int i = 0; i < size; i++) el[i] = x[i]; //res[i] = x[i];

		return *this; //res;
	}

	//
	//	Method to print  all the vector elements
	//
	template<class T, size_t size>
	inline void _vector<T,size>::printv() {
		for (int i = 0; i < size; i++) {
			Serial.print(el[i]);
			Serial.print("\t");
		}
		Serial.println();
	}

	//
	//	Vector Sum implementation
	//
	//template<class T>
	//_vector<T> _vector<T>::operator+(const _vector<T> &a, const _vector<T> &b) {
	template<class T, size_t size> _vector<T, size> operator+ (const _vector<T,size> &x, const _vector<T, size> &y) {
		_vector<T, size> res();

		for(int i = 0; i < size; i++) res[i] = x[i] + y[i];

		return res;
	}

	//
	//	Vector Difference implementation
	//
	//template<class T>
	//_vector<T> _vector<T>::operator+(const _vector<T> &a, const _vector<T> &b) {
	template<class T, size_t size> _vector<T, size> operator- (const _vector<T,size> &x, const _vector<T, size> &y) {
		_vector<T,size> res();

		for(int i = 0; i < size; i++) res[i] = x[i] - y[i];

		return res;
	}

	//
	//	Vector Scaling by factor implementation
	//
	//template<class T>
	//_vector<T> _vector<T>::operator*(const _vector<T> &v, const T a) {
	template<class T, size_t size> _vector<T,size> operator* (const _vector<T,size> &x, const T &y) {
		_vector<T,size> res();

		for(int i = 0; i < size; i++) res[i] = y * x[i];

		return res;
	}

	//
	//	Vector by Vector multiplication implementation
	//
	//template<class T>
	//T _vector<T>::operator*(const _vector<T> &a, const _vector<T> &b) {
	template<class T, size_t size> T operator* (const _vector<T,size> &x, const _vector<T, size> &y) {
		T res;

		for(int i = 0; i < size; i++) res += x[i] * y[i];

		return res;
	}

	//
	//	Vector element setting implementation
	//
	template<class T, size_t size>
	void _vector<T,size>::setv(const _vector<T,size> &v) {
		for(int i = 0; i < size; i++) el[i] = v[i];
	}

	template<class T, size_t N, size_t M>
	inline _matrix<T, N, M>::_matrix() {
		//row = n;
		//col = m;
		//el = new T[N][M];

		for(int i = 0; i < N; i++)
			for(int j = 0; j < M; j++) el[i][j] = 0;
	}

	template<class T, size_t N, size_t M>
	inline _matrix<T, N, M>::_matrix(const _matrix<T, N, M>& m) {
		//row = n;
		//col = m;
		//el = new T[row][col];

		for(int i = 0; i < N; i++)
			for(int j = 0; j < M; j++) el[i][j] = m[i][j];
	}

	template<class T, size_t N, size_t M>
	inline void _matrix<T,N,M>::printm() {
		for(int i = 0; i < N; i++) {
			for(int j = 0; j < M; j++) {
				Serial.print(el[i][j]);
				Serial.print("\t");
			}
			Serial.println();
		}
	}
	/*
*/
}


#endif /* MATH_COMP_H_ */
