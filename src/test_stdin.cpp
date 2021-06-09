#include<iostream>
#include<fstream>
using namespace std;

const char file_name[]="test1.txt";

int main()
{
    ifstream infile(file_name);
    // freopen(file_name, "r", stdin);
    int T;
    // cin>>T;
    infile>>T;
    int X,Y,Z;
    int spx, spy, spz, epx, epy, epz;
    // cin>>X>>Y>>Z;
    // cin>>spx>>spy>>spz>>epx>>epy>>epz;
    infile>>X>>Y>>Z;
    infile>>spx>>spy>>spz>>epx>>epy>>epz;
    cout<<"X Y Z  "<<X<<"\t"<<Y<<"\t"<<Z<<endl;
    cout<<spx<<"\t"<<spy<<"\t"<<spz<<endl;
    cout<<epx<<"\t"<<epy<<"\t"<<epz<<endl;
    return 0;
}