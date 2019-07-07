class average{
	private:
		int* loc;
		int ar;
	public:
		average(int*,int);
		float update(int);
};

average::average(int* in, int a){
	loc = in;
	ar = a;
	for(int i=0;i<ar;i++)
		loc[i] = 0;
}

float average::update(int in){
	int temp[ar];
	
	int low1 = 0; 
	int low2 = 0;
	int high1 = 0; 
	int high2 = 0;
	int sum =0;
	
	//initalization to low and high making sure the 
	//values are entered in the correct order
	if (loc[1]<=loc[2])
	{
		low1 = loc[1];
		low2 = loc[2];
		high2 = loc[1];
		high1 = loc[2];
	}
	//fliped if values are flipped
	else
	{
		low1 = loc[2];
		low2 = loc[1];
		high2 = loc[2];
		high1 = loc[1];
	}
	
	//shift lower by one
	for(int i=1;i<ar;i++)
	{
		temp[i-1] = loc[i];
	}
	// add new value
	temp[ar-1] = in;
	
	
	//run summation
	for(int i=0;i<ar;i++){
		sum = sum+temp[i];
		//Serial.println(temp[i]);
		//remove lowest and highest values
		if(temp[i] < low1)
		{
			low2 = low1;
			low1 = temp[i];
		}
		else if(temp[i] > high1)
		{
			high2 = high1;
			high1 = temp[i];
		}
	}
	

	//Update the proper array
	for(int i=0;i<ar;i++) 
	{
		loc[i] = temp[i];
	}
	
	sum = sum - (low1 + low2 + high1 + high2);
	return float(sum)/(float(ar)-4.0);
}