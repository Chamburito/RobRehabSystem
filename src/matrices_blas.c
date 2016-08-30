////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "matrices.h"

// Fortran77 function declarations

// (BLAS) matrix-matrix product
extern void dgemm_( char* tA, char* tB, int* m, int* n, int* k, double* alpha, double* A, int* ldA, double* B, int* ldB, double* beta, double* C, int* ldC );  
// (LAPACK) LU decomoposition of a general matrix
extern void dgetrf_( int* M, int *N, double* A, int* ldA, int* IPIV, int* INFO );
// (LAPACK) generate inverse of a matrix given its LU decomposition
extern void dgetri_( int* N, double* A, int* ldA, int* IPIV, double* WORK, int* lwork, int* INFO );


struct _MatrixData
{
  double* data;
  size_t rowsNumber, columnsNumber;
};

DEFINE_NAMESPACE_INTERFACE( Matrices, MATRICES_INTERFACE )


Matrix Matrices_Create( double* data, size_t rowsNumber, size_t columnsNumber )
{
  if( rowsNumber > MATRIX_SIZE_MAX || columnsNumber > MATRIX_SIZE_MAX ) return NULL;

  Matrix newMatrix = (Matrix) malloc( sizeof(MatrixData) );

  newMatrix->data = (double*) calloc( rowsNumber * columnsNumber, sizeof(double) );

  newMatrix->rowsNumber = rowsNumber;
  newMatrix->columnsNumber = columnsNumber;

  if( data == NULL ) Matrices_Clear( newMatrix );
  else Matrices_SetData( newMatrix, data );

  return newMatrix;
}

Matrix Matrices_CreateSquare( size_t size, char type )
{
  Matrix newSquareMatrix = Matrices_Create( NULL, size, size );

  if( type == 'I' )
  {
    for( size_t line = 0; line < size; line++ )
      newSquareMatrix->data[ line * size + line ] = 1.0;
  }

  return newSquareMatrix;
}

void Matrices_Discard( Matrix matrix )
{
  if( matrix == NULL ) return;
  
  free( matrix->data );
  
  free( matrix );
}

Matrix Matrices_Copy( Matrix source, Matrix destination )
{
  if( source == NULL || destination == NULL ) return NULL;

  destination->rowsNumber = source->rowsNumber;
  destination->columnsNumber = source->columnsNumber;

  memcpy( destination->data, source->data, destination->rowsNumber * destination->columnsNumber * sizeof(double) );

  return destination;
}

Matrix Matrices_Clear( Matrix matrix )
{
  if( matrix == NULL ) return NULL;

  memset( matrix->data, 0, matrix->rowsNumber * matrix->columnsNumber * sizeof(double) );

  return matrix;
}

size_t Matrices_GetWidth( Matrix matrix )
{
  if( matrix == NULL ) return 0;

  return matrix->columnsNumber;
}

size_t Matrices_GetHeight( Matrix matrix )
{
  if( matrix == NULL ) return 0;

  return matrix->rowsNumber;
}

double Matrices_GetElement( Matrix matrix, size_t row, size_t column )
{
  if( matrix == NULL ) return 0.0;

  if( row >= matrix->rowsNumber || column >= matrix->columnsNumber ) return 0.0;

  return matrix->data[ column * matrix->rowsNumber + row ];
}

void Matrices_SetElement( Matrix matrix, size_t row, size_t column, double value )
{
  if( matrix == NULL ) return;

  if( row >= matrix->rowsNumber || column >= matrix->columnsNumber ) return;

  matrix->data[ column * matrix->rowsNumber + row ] = value;
}

double* Matrices_GetData( Matrix matrix, double* buffer )
{
  if( matrix == NULL ) return NULL;

  for( size_t row = 0; row < matrix->rowsNumber; row++ )
  {
    for( size_t column = 0; column < matrix->columnsNumber; column++ )
      buffer[ row * matrix->columnsNumber + column ] = matrix->data[ column * matrix->rowsNumber + row ];
  }
  
  return buffer;
}

void Matrices_SetData( Matrix matrix, double* data )
{
  if( matrix == NULL ) return;

  for( size_t column = 0; column < matrix->columnsNumber; column++ )
  {
    for( size_t row = 0; row < matrix->rowsNumber; row++ )
      matrix->data[ column * matrix->rowsNumber + row ] = data[ row * matrix->columnsNumber + column ];
  }
}

Matrix Matrices_Resize( Matrix matrix, size_t rowsNumber, size_t columnsNumber )
{
  double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];
  
  if( matrix == NULL )
    matrix = Matrices_Create( NULL, rowsNumber, columnsNumber );
  else 
  {
    if( matrix->rowsNumber * matrix->columnsNumber < rowsNumber * columnsNumber )
      matrix->data = (double*) realloc( matrix->data, rowsNumber * columnsNumber * sizeof(double) );
  
    memcpy( auxArray, matrix->data, matrix->rowsNumber * matrix->columnsNumber * sizeof(double) );
    
    memset( matrix->data, 0, rowsNumber * columnsNumber * sizeof(double) );
    
    for( size_t column = 0; column < matrix->columnsNumber; column++ )
    {
      for( size_t row = 0; row < matrix->rowsNumber; row++ )
        matrix->data[ column * rowsNumber + row ] = auxArray[ column * matrix->rowsNumber + row ];
    }
    
    matrix->rowsNumber = rowsNumber;
    matrix->columnsNumber = columnsNumber;
  }

  return matrix;
}

Matrix Matrices_Scale( Matrix matrix, double scalar, Matrix result )
{
  if( matrix == NULL ) return NULL;
  
  size_t elementsNumber = result->rowsNumber * result->columnsNumber;
  for( size_t elementIndex = 0; elementIndex < elementsNumber; elementIndex++ )
    result->data[ elementIndex ] = scalar * matrix->data[ elementIndex ];

  result->rowsNumber = matrix->rowsNumber;
  result->columnsNumber = matrix->columnsNumber;
  
  return result;
}

Matrix Matrices_Sum( Matrix matrix_1, double weight_1, Matrix matrix_2, double weight_2, Matrix result )
{
  if( matrix_1 == NULL || matrix_2 == NULL ) return NULL;

  if( matrix_1->rowsNumber != matrix_2->rowsNumber || matrix_1->columnsNumber != matrix_2->columnsNumber ) return NULL;

  result->rowsNumber = matrix_1->rowsNumber;
  result->columnsNumber = matrix_1->columnsNumber;
  
  size_t elementsNumber = result->rowsNumber * result->columnsNumber;
  for( size_t elementIndex = 0; elementIndex < elementsNumber; elementIndex++ )
    result->data[ elementIndex ] = weight_1 * matrix_1->data[ elementIndex ] + weight_2 * matrix_2->data[ elementIndex ];

  return result;
}

Matrix Matrices_Dot( Matrix matrix_1, char transpose_1, Matrix matrix_2, char transpose_2, Matrix result )
{
  const double alpha = 1.0;
  const double beta = 0.0;
  
  double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];
  
  if( matrix_1 == NULL || matrix_2 == NULL ) return NULL;
  
  size_t couplingLength = ( transpose_1 == MATRIX_TRANSPOSE ) ? matrix_1->rowsNumber : matrix_1->columnsNumber;
   
  if( couplingLength != ( ( transpose_2 == MATRIX_TRANSPOSE ) ? matrix_2->columnsNumber : matrix_2->rowsNumber ) ) return NULL;
   
  result->rowsNumber = ( transpose_1 == MATRIX_TRANSPOSE ) ? matrix_1->columnsNumber : matrix_1->rowsNumber;
  result->columnsNumber = ( transpose_2 == MATRIX_TRANSPOSE ) ? matrix_2->rowsNumber : matrix_2->columnsNumber;
  
  int stride_1 = ( transpose_1 == MATRIX_TRANSPOSE ) ? couplingLength : result->rowsNumber;          // Distance between columns
  int stride_2 = ( transpose_2 == MATRIX_TRANSPOSE ) ? result->columnsNumber : couplingLength;       // Distance between columns
  
  dgemm_( &transpose_1, &transpose_2, (int*) &(result->rowsNumber),(int*) &(result->columnsNumber), (int*) &(couplingLength), 
          (double*) &alpha, matrix_1->data, &stride_1, matrix_2->data, &stride_2, (double*) &beta, auxArray, (int*) &result->rowsNumber );
  
  memcpy( result->data, auxArray, result->rowsNumber * result->columnsNumber * sizeof(double) );

  return result;
}

double Matrices_Determinant( Matrix matrix )
{
  double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];
  int pivotArray[ MATRIX_SIZE_MAX ];
  int info;
  
  if( matrix == NULL ) return 0.0;

  if( matrix->rowsNumber != matrix->columnsNumber ) return 0.0;
  
  memcpy( auxArray, matrix->data, matrix->rowsNumber * matrix->columnsNumber * sizeof(double) );
  
  dgetrf_( (int*) &(matrix->rowsNumber), (int*) &(matrix->columnsNumber), auxArray, (int*) &(matrix->rowsNumber), pivotArray, &info );
  
  double determinant = 1.0;
  for( size_t pivotIndex = 0; pivotIndex < matrix->rowsNumber; pivotIndex++ )
  {
    determinant *= auxArray[ pivotIndex * matrix->rowsNumber + pivotIndex ];
    if( pivotArray[ pivotIndex ] != pivotIndex ) determinant *= -1.0;
  }

  return determinant;
}

Matrix Matrices_Transpose( Matrix matrix, Matrix result )
{
  double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ] = { 0 };
  
  if( matrix == NULL ) return NULL;

  result->rowsNumber = matrix->columnsNumber;
  result->columnsNumber = matrix->rowsNumber;

  for( size_t row = 0; row < result->rowsNumber; row++ )
  {
    for( size_t column = 0; column < result->columnsNumber; column++ )
      auxArray[ column * result->rowsNumber + row ] = matrix->data[ row * matrix->rowsNumber + column ];
  }

  memcpy( result->data, auxArray, result->rowsNumber * result->columnsNumber * sizeof(double) );

  return result;
}

Matrix Matrices_Inverse( Matrix matrix, Matrix result )
{
  double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ] = { 0 };
  int pivotArray[ MATRIX_SIZE_MAX ];
  int info;
  
  if( matrix == NULL || result == NULL ) return NULL;

  if( matrix->rowsNumber != matrix->columnsNumber ) return NULL;

  if( matrix != result )
  {
    result->rowsNumber = matrix->rowsNumber;
    result->columnsNumber = matrix->columnsNumber;
  
    memcpy( result->data, matrix->data, matrix->rowsNumber * matrix->columnsNumber * sizeof(double) );
  }
  
  dgetrf_( (int*) &(result->rowsNumber), (int*) &(result->columnsNumber), result->data, (int*) &(result->rowsNumber), pivotArray, &info );
  
  if( info != 0 ) return NULL;
  
  int workLength = result->rowsNumber * result->columnsNumber;
  dgetri_( (int*) &(result->rowsNumber), result->data, (int*) &(result->rowsNumber), pivotArray, auxArray, &workLength, &info );
  
  if( info != 0 ) return NULL;

  return result;
}

void Matrices_Print( Matrix matrix )
{
  if( matrix == NULL ) return;

  printf( "[%lux%lu] matrix:\n", matrix->rowsNumber, matrix->columnsNumber );
  for( size_t row = 0; row < matrix->rowsNumber; row++ )
  {
    printf( "[" );
    for( size_t column = 0; column < matrix->columnsNumber; column++ )
      printf( " %.6f", matrix->data[ column * matrix->rowsNumber + row ] );
    printf( " ]\n" );
  }
  printf( "\n" );
}
