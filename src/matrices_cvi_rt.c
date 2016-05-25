#include "matrices.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <analysis.h>


struct _MatrixData
{
  double* data;
  size_t rowsNumber, columnsNumber;
};

DEFINE_NAMESPACE_INTERFACE( Matrices, MATRICES_INTERFACE )


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
  else Matrices_SetData( newMatrix, data );

  return newMatrix;
}

Matrix Matrices_Copy( Matrix source, Matrix destination )
{
  if( source == NULL || destination == NULL ) return NULL;

  destination->rowsNumber = source->rowsNumber;
  destination->columnsNumber = source->columnsNumber;

  memcpy( destination->data, source->data, destination->rowsNumber * destination->columnsNumber * sizeof(double) );

  return destination;
}

// Atalho para criar matriz quadrada (zero ou identidade)
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
  double auxArray[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];
  
  if( matrix == NULL )
    matrix = Matrices_Create( NULL, rowsNumber, columnsNumber );
  else 
  {
    if( matrix->rowsNumber * matrix->columnsNumber < rowsNumber * columnsNumber )
      matrix->data = (double*) realloc( matrix->data, rowsNumber * columnsNumber * sizeof(double) );
  
    memcpy( auxArray, matrix->data, matrix->rowsNumber * matrix->columnsNumber * sizeof(double) );
    
    memset( matrix->data, 0, rowsNumber * columnsNumber * sizeof(double) );
    
    for( size_t row = 0; row < rowsNumber; row++ )
    {
      for( size_t column = 0; column < columnsNumber; column++ )
        matrix->data[ row * columnsNumber + column ] = auxArray[ row * matrix->columnsNumber + column ];
    }
    
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

// Escreve elemento na matriz na linha e coluna dadas
void Matrices_SetElement( Matrix matrix, size_t row, size_t column, double value )
{
  if( matrix == NULL ) return;

  if( row >= matrix->rowsNumber || column >= matrix->columnsNumber ) return;

  matrix->data[ row * matrix->columnsNumber + column ] = value;
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

// Obtém vetor de 1 dimensão dos elementos da matriz
double* Matrices_GetData( Matrix matrix, double* buffer )
{
  if( matrix == NULL ) return NULL;

  memcpy( buffer, matrix->data, matrix->rowsNumber * matrix->columnsNumber * sizeof(double) );
  
  return buffer;
}

void Matrices_SetData( Matrix matrix, double* data )
{
  if( matrix == NULL ) return;

  memcpy( matrix->data, data, matrix->rowsNumber * matrix->columnsNumber * sizeof(double) );
}

// Multiplica matriz por escalar
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

// Soma de matrizes
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

// Produto de matrizes
Matrix Matrices_Dot( Matrix matrix_1, char transpose_1, Matrix matrix_2, char transpose_2, Matrix result )
{
  double auxArray_1[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];
  double auxArray_2[ MATRIX_SIZE_MAX * MATRIX_SIZE_MAX ];
  
  if( matrix_1 == NULL || matrix_2 == NULL ) return NULL;
  
  size_t couplingLength = ( transpose_1 == MATRIX_TRANSPOSE ) ? matrix_1->rowsNumber : matrix_1->columnsNumber;
   
  if( couplingLength != ( ( transpose_2 == MATRIX_TRANSPOSE ) ? matrix_2->columnsNumber : matrix_2->rowsNumber ) ) return NULL;
   
  result->rowsNumber = ( transpose_1 == MATRIX_TRANSPOSE ) ? matrix_1->columnsNumber : matrix_1->rowsNumber;
  result->columnsNumber = ( transpose_2 == MATRIX_TRANSPOSE ) ? matrix_2->rowsNumber : matrix_2->columnsNumber;
  
  if( transpose_1 == MATRIX_TRANSPOSE ) Transpose( matrix_1->data, matrix_1->rowsNumber, matrix_1->columnsNumber, auxArray_1 );
  else memcpy( auxArray_1, matrix_1->data, matrix_1->rowsNumber * matrix_1->columnsNumber * sizeof(double) );
  
  if( transpose_2 == MATRIX_TRANSPOSE ) Transpose( matrix_2->data, matrix_2->rowsNumber, matrix_2->columnsNumber, auxArray_2 );
  else memcpy( auxArray_2, matrix_2->data, matrix_2->rowsNumber * matrix_2->columnsNumber * sizeof(double) );
  
  if( MatrixMul( auxArray_1, auxArray_2, result->rowsNumber, couplingLength, result->columnsNumber, result->data ) != NoAnlysErr ) return NULL;

  return result;
}

// Determinante de matriz
double Matrices_Determinant( Matrix matrix )
{
  if( matrix == NULL ) return 0.0;

  if( matrix->rowsNumber != matrix->columnsNumber ) return 0.0;
  
  double determinant;
  if( Determinant( matrix->data, matrix->rowsNumber, &determinant ) != NoAnlysErr ) return 0.0;

  return determinant;
}

// Matriz transposta
Matrix Matrices_Transpose( Matrix matrix, Matrix result )
{
  if( matrix == NULL ) return NULL;

  result->rowsNumber = matrix->columnsNumber;
  result->columnsNumber = matrix->rowsNumber;

  if( Transpose( matrix->data, matrix->rowsNumber, matrix->columnsNumber, result->data ) != NoAnlysErr ) return NULL;

  return result;
}

// Matriz inversa
Matrix Matrices_Inverse( Matrix matrix, Matrix result )
{
  if( matrix == NULL || result == NULL ) return NULL;

  if( matrix->rowsNumber != matrix->columnsNumber ) return NULL;

  if( matrix != result )
  {
    result->rowsNumber = matrix->rowsNumber;
    result->columnsNumber = matrix->columnsNumber;
  }
  
  if( InvMatrix( matrix->data, result->rowsNumber, result->data ) != NoAnlysErr ) return NULL;

  return result;
}

// Imprime matriz
void Matrices_Print( Matrix matrix )
{
  if( matrix == NULL ) return;

  fprintf( stderr, "[%lux%lu] matrix:\n", matrix->rowsNumber, matrix->columnsNumber );
  for( size_t row = 0; row < matrix->rowsNumber; row++ )
  {
    fprintf( stderr, "[" );
    for( size_t column = 0; column < matrix->columnsNumber; column++ )
      fprintf( stderr, " %.6f", matrix->data[ row * matrix->columnsNumber + column ] );
    fprintf( stderr, " ]\n" );
  }
  fprintf( stderr, "\n" );
}
