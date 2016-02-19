#ifndef MATRICES_H
#define MATRICES_H

#include "interfaces.h"
#include "debug/async_debug.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define MATRIX_SIZE_MAX 50 

#define MATRIX_IDENTITY true
#define MATRIX_ZERO false

#define MATRIX_TRANSPOSE true
#define MATRIX_KEEP false

typedef struct _MatrixData
{
  double* data;
  size_t rowsNumber, columnsNumber;
}
MatrixData;

typedef MatrixData* Matrix;


#define MATRICES_FUNCTIONS( namespace, function_init ) \
        function_init( Matrix, namespace, Create, double*, size_t, size_t ) \               
        function_init( Matrix, namespace, CreateSquare, size_t, bool ) \                    
        function_init( double, namespace, GetElement, Matrix, size_t, size_t ) \            
        function_init( double*, namespace, GetAsVector, Matrix ) \                             
        function_init( void, namespace, SetElement, Matrix, size_t, size_t, double ) \      
        function_init( Matrix, namespace, Clear, Matrix ) \                                 
        function_init( Matrix, namespace, Resize, Matrix, size_t, size_t ) \                
        function_init( Matrix, namespace, Scale, Matrix, double, Matrix ) \                 
        function_init( Matrix, namespace, Sum, Matrix, double, Matrix, double, Matrix ) \       
        function_init( Matrix, namespace, Dot, Matrix, bool, Matrix, bool, Matrix ) \       
        function_init( double, namespace, Determinant, Matrix ) \                           
        function_init( Matrix, namespace, Transpose, Matrix, Matrix ) \                     
        function_init( Matrix, namespace, Inverse, Matrix, Matrix ) \                       
        function_init( void, namespace, Print, Matrix )                                     

INIT_NAMESPACE_INTERFACE( Matrices, MATRICES_FUNCTIONS )


// Preenche a matrix com zeros
Matrix Matrices_Clear( Matrix matrix )
{
  if( matrix == NULL ) return NULL;

  memset( matrix->data, 0, matrix->rowsNumber * matrix->columnsNumber * sizeof(double) );

  return matrix;
}

// Criar a partir de array de tamanho linhas * colunas
Matrix Matrices_Create( double* data, size_t rowsNumber, size_t columnsNumber )
{
  if( rowsNumber > MATRIX_SIZE_MAX || columnsNumber > MATRIX_SIZE_MAX ) return NULL;

  Matrix newMatrix = (Matrix) malloc( sizeof(MatrixData) );

  newMatrix->data = (double*) calloc( rowsNumber * columnsNumber, sizeof(double) );

  newMatrix->rowsNumber = rowsNumber;
  newMatrix->columnsNumber = columnsNumber;

  if( data == NULL ) Matrices_Clear( newMatrix );
  else memcpy( newMatrix->data, data, rowsNumber * columnsNumber * sizeof(double) );

  return newMatrix;
}

// Atalho para criar matriz quadrada (zero ou identidade)
Matrix Matrices_CreateSquare( size_t size, bool isIdentity )
{
  Matrix newSquareMatrix = Matrices_Create( NULL, size, size );

  if( isIdentity )
  {
    for( size_t line = 0; line < size; line++ )
      newSquareMatrix->data[ line * size + line ] = 1.0;
  }

  return newSquareMatrix;
}

// Desaloca memória de matrix fornecida
void Matrices_Discard( Matrix matrix )
{
  if( matrix == NULL ) return;
  
  free( matrix->data );
  
  free( matrix );
}

// Realoca a matriz para tamanho (linhas e colunas) dado
Matrix Matrices_Resize( Matrix matrix, size_t rowsNumber, size_t columnsNumber )
{
  if( matrix == NULL )
    matrix = Matrices_Create( NULL, rowsNumber, columnsNumber );
  else if( matrix->rowsNumber < rowsNumber || matrix->columnsNumber < columnsNumber )
  {
    matrix->data = (double*) realloc( matrix->data, rowsNumber * columnsNumber * sizeof(double) );
    matrix->rowsNumber = rowsNumber;
    matrix->columnsNumber = columnsNumber;
  }

  return matrix;
}

// Obtém elemento da matriz na linha e coluna dadas
double Matrices_GetElement( Matrix matrix, size_t row, size_t column )
{
  if( matrix == NULL ) return 0.0;

  if( row >= matrix->rowsNumber || column >= matrix->columnsNumber ) return 0.0;

  return matrix->data[ row * matrix->columnsNumber + column ];
}

// Obtém vetor de 1 dimensão dos elementos da matriz
double* Matrices_GetAsVector( Matrix matrix )
{
  if( matrix == NULL ) return NULL;

  return matrix->data;
}

// Escreve elemento na matriz na linha e coluna dadas
void Matrices_SetElement( Matrix matrix, size_t row, size_t column, double value )
{
  if( matrix == NULL ) return;

  if( row >= matrix->rowsNumber || column >= matrix->columnsNumber ) return;

  matrix->data[ row * matrix->columnsNumber + column ] = value;
}

// Multiplica matriz por escalar
Matrix Matrices_Scale( Matrix matrix, double scalar, Matrix result )
{
  if( matrix == NULL ) return NULL;
  
  if( (result = Matrices_Resize( result, matrix->rowsNumber, matrix->columnsNumber )) == NULL ) return NULL;

  for( size_t row = 0; row < matrix->rowsNumber; row++ )
  {
    for( size_t column = 0; column < matrix->columnsNumber; column++ )
      result->data[ row * matrix->columnsNumber + column ] = matrix->data[ row * matrix->columnsNumber + column ] * scalar;
  }

  return result;
}

// Soma de matrizes
Matrix Matrices_Sum( Matrix matrix_1, double weight_1, Matrix matrix_2, double weight_2, Matrix result )
{
  static double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];

  if( matrix_1 == NULL || matrix_2 == NULL ) return NULL;

  if( matrix_1->rowsNumber != matrix_2->rowsNumber || matrix_1->columnsNumber != matrix_2->columnsNumber ) return NULL;

  if( (result = Matrices_Resize( result, matrix_1->rowsNumber, matrix_1->columnsNumber )) == NULL ) return NULL;

  for( size_t row = 0; row < result->rowsNumber; row++ )
  {
    for( size_t column = 0; column < result->columnsNumber; column++ )
    {
      double element_1 = matrix_1->data[ row * matrix_1->columnsNumber + column ];
      double element_2 = matrix_2->data[ row * matrix_2->columnsNumber + column ];
      auxArray[ row * result->columnsNumber + column ] = weight_1 * element_1 + weight_2 * element_2;
    }
  }

  memcpy( result->data, auxArray, result->rowsNumber * result->columnsNumber * sizeof(double) );

  return result;
}

// Produto de matrizes
Matrix Matrices_Dot( Matrix matrix_1, bool transpose_1, Matrix matrix_2, bool transpose_2, Matrix result )
{
  static double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];

  if( matrix_1 == NULL || matrix_2 == NULL ) return NULL;

  size_t couplingLength = transpose_1 ? matrix_1->rowsNumber : matrix_1->columnsNumber;
  
  if( couplingLength != ( transpose_2 ? matrix_2->columnsNumber : matrix_2->rowsNumber ) ) return NULL;
  
  size_t resultRowsNumber = transpose_1 ? matrix_1->columnsNumber : matrix_1->rowsNumber;
  size_t resultColumnsNumber = transpose_2 ? matrix_2->rowsNumber : matrix_2->columnsNumber;

  if( (result = Matrices_Resize( result, resultRowsNumber, resultColumnsNumber )) == NULL ) return NULL;

  for( size_t row = 0; row < result->rowsNumber; row++ )
  {
    for( size_t column = 0; column < result->columnsNumber; column++ )
    {
      auxArray[ row * result->columnsNumber + column ] = 0.0;
      for( size_t i = 0; i < couplingLength; i++ )
      {
        size_t elementIndex_1 = transpose_1 ? i * matrix_1->columnsNumber + row : row * matrix_1->columnsNumber + i;
        size_t elementIndex_2 = transpose_2 ? column * matrix_2->columnsNumber + i : i * matrix_2->columnsNumber + column;
        auxArray[ row * result->columnsNumber + column ] += matrix_1->data[ elementIndex_1 ] * matrix_2->data[ elementIndex_2 ];
      }
    }
  }

  memcpy( result->data, auxArray, result->rowsNumber * result->columnsNumber * sizeof(double) );

  return result;
}

// NÃO USAR DIRETAMENTE !!
void CofactorArray( double* matrixArray, size_t size, size_t elementRow, size_t elementColumn, double* result )
{
  size_t cofRow = 0, cofColumn = 0;
  size_t cofSize = size - 1;
  for( size_t row = 0; row < size; row++ )
  {
    for( size_t column = 0; column < size; column++ )
    {
      if( row != elementRow && column != elementColumn )
      {
        result[ cofRow * cofSize + cofColumn ] = matrixArray[ row * size + column ];

        if( cofColumn < cofSize - 1 ) cofColumn++;
        else
        {
          cofColumn = 0;
          cofRow++;
        }
      }
    }
  }
}

// NÃO USAR DIRETAMENTE !!
double ArrayDeterminant( double* matrixArray, size_t size )
{
  double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];

  if( size == 1 ) return matrixArray[ 0 ];

  double result = 0.0;
  for( size_t c = 0; c < size; c++ )
  {
    CofactorArray( matrixArray, size, 0, c, (double*) auxArray );
    result += pow( -1, c ) * matrixArray[ c ] * ArrayDeterminant( (double*) auxArray, size - 1 );
  }

  return result;
}

// NÃO USAR DIRETAMENTE !!
double ArrayCofactor( double* matrixArray, size_t size, size_t row, size_t column )
{
  double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];

  CofactorArray( matrixArray, size, row, column, (double*) auxArray );

  double result = pow( -1, row + column ) * ArrayDeterminant( (double*) auxArray, size - 1 );

  return result;
}

// Determinante de matriz
double Matrices_Determinant( Matrix matrix )
{
  if( matrix == NULL ) return 0.0;

  if( matrix->rowsNumber != matrix->columnsNumber ) return 0.0;

  return ArrayDeterminant( matrix->data, matrix->rowsNumber );
}

// Matriz transposta
Matrix Matrices_Transpose( Matrix matrix, Matrix result )
{
  static double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];

  if( matrix == NULL ) return NULL;

  if( (result = Matrices_Resize( result, matrix->columnsNumber, matrix->rowsNumber )) == NULL ) return NULL;

  for( size_t row = 0; row < result->rowsNumber; row++ )
  {
    for( size_t column = 0; column < result->columnsNumber; column++ )
      auxArray[ row * matrix->columnsNumber + column ] = matrix->data[ column * matrix->columnsNumber + row ];
  }

  memcpy( result->data, auxArray, result->rowsNumber * result->columnsNumber * sizeof(double) );

  return result;
}

// Matriz inversa
Matrix Matrices_Inverse( Matrix matrix, Matrix result )
{
  static double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];

  double determinant = Matrices_Determinant( matrix );

  if( determinant == 0.0 ) return NULL;

  if( (result = Matrices_Resize( result, matrix->columnsNumber, matrix->rowsNumber )) == NULL ) return NULL;

  for( size_t row = 0; row < result->rowsNumber; row++ )
  {
    for( size_t column = 0; column < result->columnsNumber; column++ )
      auxArray[ row * matrix->columnsNumber + column ] = ArrayCofactor( matrix->data, result->rowsNumber, column, row ) / determinant;
  }

  memcpy( result->data, auxArray, result->rowsNumber * result->columnsNumber * sizeof(double) );

  return result;
}

// Imprime matriz
void Matrices_Print( Matrix matrix )
{
  if( matrix == NULL ) return;

  for( size_t row = 0; row < matrix->rowsNumber; row++ )
  {
    printf( "[" );
    for( size_t column = 0; column < matrix->columnsNumber; column++ )
      printf( " %.3f", matrix->data[ row * matrix->columnsNumber + column ] );
    printf( " ]\n" );
  }
   printf( "\n" );
}

#endif // MATRICES_H
