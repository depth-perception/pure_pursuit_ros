#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fstream>
#include <math.h>
#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{
  ofstream file;
  file.open("./path_data.txt",ios::out);
  if(!file.is_open()){
    cout << "Open file faile!" << endl;
  }

  double x,y;
  for(int i=0; i<100; i++){
    x = 100 - i;
    y = sin(x/15.0) * x / 2.0;
    file << x << "\t "<< y << endl;
  }

  file.close();
  printf("Path has been created!\n");

  return 0;
}
