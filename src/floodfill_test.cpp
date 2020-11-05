// Including the input and output stream
#include <iostream>
#include <vector>

using namespace std;

unsigned int matrix[5][5] = {0};


int flood_fill(unsigned int c, unsigned int r,
               unsigned int target_value,
               unsigned int replacement)
  {
    if (target_value == replacement)
    {
        return 0;
    }
      
    else if (matrix[c][r] != target_value)
    {
        return 0;
    }
      
    else
    {
        matrix[c][r] = replacement;
    }
    if (c<0)
    {
        flood_fill(c-1, r,   target_value, replacement);
    }
    
    if (c<5)
    {
        flood_fill(c+1, r,   target_value, replacement);
    }
    
    if (r<0)
    {
        flood_fill(c,   r-1, target_value, replacement);
    }
    
    if (r<5)
    {
        flood_fill(c,   r+1, target_value, replacement);
    }

    if( (c<0) && (r<0) )
    {
        flood_fill(c-1,   r-1, target_value, replacement);
    }
    
    if( (c<5) && (r<0) )
    {
        flood_fill(c+1,   r-1, target_value, replacement);
    }
    if( (c<0) && (r<5) )
    {
        flood_fill(c-1,   r+1, target_value, replacement);
    }
    if( (c<5) && (r<5) )
    {
        flood_fill(c+1,   r+1, target_value, replacement);
    }

    return 0;
  }


int main()
{ 
  int x = 0;
  int y = 0;
  int pixel = 87815;
  uint32_t width=  384;

  y = pixel / width;
  x = pixel % width;

  cout << "x: " << x << endl;
  cout << "y: " << y << endl;
  // unsigned int ausgabe=0;
  // matrix[0][0] = 1;
  // matrix[1][1] = 1;
  // matrix[2][2] = 1;
  // matrix[3][3] = 1;
  // matrix[4][4] = 1;
  // matrix[0][4] = 1;
  // matrix[1][3] = 1;
  // matrix[3][1] = 1;
  // matrix[4][0] = 1;
  
  // for(unsigned int c=0; c<5 ; c++)
  // {
  //   for(unsigned int r=0; r<5 ; r++ )
  //   { 
  //     ausgabe= matrix[c][r];
  //     cout<< ausgabe ;
  //   }
  //   cout<<"\n";
  // }

  //   cout<<"\n";

  //   for(unsigned int c=0; c<5 ; c++)
  // {
  //   for(unsigned int r=0; r<5 ; r++ )
  //   { 
  //     flood_fill(c,r,1,4);
  //     ausgabe= matrix[c][r];
  //     cout<< ausgabe ;
  //   }
  //   cout<<"\n";
  // }


  return 0;
}
