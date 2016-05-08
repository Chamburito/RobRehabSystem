#ifndef MATRICES_H
#define MATRICES_H

#include "namespaces.h"

#define MATRIX_SIZE_MAX 50 

#define MATRIX_IDENTITY 'I'
#define MATRIX_ZERO '0'

#define MATRIX_TRANSPOSE 'T'
#define MATRIX_KEEP 'N'


typedef struct _MatrixData MatrixData;
typedef MatrixData* Matrix;

#define MATRICES_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( Matrix, Namespace, Create, double*, size_t, size_t ) \
        INIT_FUNCTION( Matrix, Namespace, CreateSquare, size_t, char ) \
        INIT_FUNCTION( void, Namespace, Discard, Matrix ) \
        INIT_FUNCTION( Matrix, Namespace, Copy, Matrix, Matrix ) \
        INIT_FUNCTION( size_t, Namespace, GetWidth, Matrix ) \
        INIT_FUNCTION( size_t, Namespace, GetHeight, Matrix ) \
        INIT_FUNCTION( double*, Namespace, GetData, Matrix, double* ) \
        INIT_FUNCTION( void, Namespace, SetData, Matrix, double* ) \
        INIT_FUNCTION( double, Namespace, GetElement, Matrix, size_t, size_t ) \
        INIT_FUNCTION( void, Namespace, SetElement, Matrix, size_t, size_t, double ) \
        INIT_FUNCTION( Matrix, Namespace, Clear, Matrix ) \
        INIT_FUNCTION( Matrix, Namespace, Resize, Matrix, size_t, size_t ) \
        INIT_FUNCTION( Matrix, Namespace, Scale, Matrix, double, Matrix ) \
        INIT_FUNCTION( Matrix, Namespace, Sum, Matrix, double, Matrix, double, Matrix ) \
        INIT_FUNCTION( Matrix, Namespace, Dot, Matrix, char, Matrix, char, Matrix ) \
        INIT_FUNCTION( double, Namespace, Determinant, Matrix ) \
        INIT_FUNCTION( Matrix, Namespace, Transpose, Matrix, Matrix ) \
        INIT_FUNCTION( Matrix, Namespace, Inverse, Matrix, Matrix ) \
        INIT_FUNCTION( void, Namespace, Print, Matrix )                                     

DECLARE_NAMESPACE_INTERFACE( Matrices, MATRICES_INTERFACE )


#endif // MATRICES_H
