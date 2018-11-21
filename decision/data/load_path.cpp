#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sstream>

using namespace std;

int main(int argc, char* argv[])
{
  ifstream in;
  in.open("path_data.txt",ios::in);
  if(!in.is_open())
  {
    printf("Open File failed!\n");
    exit(1);
  }
  double x,y;
  string strOne;
  while(getline(in,strOne)){
    stringstream ss;
    ss << strOne;
    ss >> x >> y;
    cout << x << "\t" << y << endl;
    // fscanf(fp,"%.2f%.2f",&x,&y);
    // printf("read line: x = %.2f, y = %.2f;\n",x,y);
  }
  in.close();
  printf("read the end of the file, stop.\n");
}
