#include "processing.h"
#include "iir.h"

void calculateShelvingCoeff(float c_alpha, Int16* output)
{
	Int16 k1, k2;
	k1 = c_alpha * 32767;
	k2  = -32768*(c_alpha);

	if(output[0] > 32767) {
		output[0] = 32767;
	}
	else {
		output[0] = k1;
	}

	output[1] = -32768;
	output[2] = 32767;

	if(output[3]  < -32768){
		output[3] = -32768;
	}
	else {
		output[3] = k2;
	}
	/* Your code here */
}

void calculatePeekCoeff(float c_alpha, float c_beta, Int16* output)
{


	output[0] = c_alpha * 32767;
	output[1] =	(-c_beta * (1 + c_alpha))*16384;
	output[2] = 32767;
	output[3] = 32767;
	output[4] = (-c_beta * (1 + c_alpha))*16384;
	output[5] = c_alpha * 32767;


	/* Your code here */
}

Int16 shelvingHP(Int16 input, Int16* coeff, Int16* x_history, Int16* y_history, Int16 k)
{

	Int16 input_a1;
	Int32 output;

	input_a1 = first_order_IIR(input, coeff, x_history, y_history);

	output = _smpy(((input>>1) + (input_a1>>1)), k) + (Int32)((input>>1) - (input_a1>>1));

	if(output > 32767) {
		output = 32767;
	}

	if(output < -32768) {
		output = -32768;
	}

	return (Int16)output;
}

Int16 shelvingLP(Int16 input, Int16* coeff, Int16* x_history, Int16* y_history, Int16 k)
{
	Int16 input_a1;
	Int32 output;

	input_a1 = first_order_IIR(input, coeff, x_history, y_history);

	output = _smpy((input>>1) - (input_a1>>1), k) + (Int32)((input>>1) + (input_a1>>1));

	if(output > 32767) {
		output = 32767;
	}

	if(output < -32768) {
		output = -32768;
	}

	return (Int16)output;
}

Int16 shelvingPeek(Int16 input, Int16* coeff, Int16* x_history, Int16* y_history, Int16 k)
{
	Int16 input_a2;
	Int32 output;

	input_a2 = second_order_IIR(input, coeff, x_history, y_history);

	output = _smpy((input>>1) - (input_a2>>1), k) + (Int32)((input>>1) + (input_a2>>1));

	if(output > 32767) {
		output = 32767;
	}

	if(output < -32768) {
		output = -32768;
	}

	return (Int16)output;
}
