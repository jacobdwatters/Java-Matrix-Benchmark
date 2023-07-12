/*
 * Copyright (c) 2009-2015, Peter Abeles. All Rights Reserved.
 *
 * This file is part of JMatrixBenchmark.
 *
 * JMatrixBenchmark is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * JMatrixBenchmark is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with JMatrixBenchmark.  If not, see <http://www.gnu.org/licenses/>.
 */

package jmatbench.flag4j;

import com.flag4j.Matrix;
import jmbench.interfaces.BenchmarkMatrix;
import jmbench.interfaces.MatrixProcessorInterface;
import jmbench.interfaces.RuntimePerformanceFactory;
import jmbench.matrix.RowMajorMatrix;

/**
 * @author Jacob Watters
 */
public class Flag4jAlgorithmFactory implements RuntimePerformanceFactory {


    @Override
    public BenchmarkMatrix create(int numRows, int numCols) {
        return wrap( new Matrix(numRows, numCols) );
    }


    @Override
    public BenchmarkMatrix wrap(Object matrix) {
        return new Flag4jBenchmarkMatrix((Matrix) matrix);
    }


    /**
     * Cholesky decomposition
     */
    @Override
    public MatrixProcessorInterface chol() {
        return null;
    }


    /**
     * LU decomposition
     */
    @Override
    public MatrixProcessorInterface lu() {
        return null;
    }


    /**
     * Singular Value Decomposition
     */
    @Override
    public MatrixProcessorInterface svd() {
        return null;
    }


    /**
     * QR Decomposition
     */
    @Override
    public MatrixProcessorInterface qr() {
        return null;
    }


    /**
     * Eigenvalue Decomposition
     */
    @Override
    public MatrixProcessorInterface eigSymm() {
        return null;
    }


    /**
     * Computes the determinant of a matrix.
     */
    @Override
    public MatrixProcessorInterface det() {
        return null;
    }


    /**
     * Inverts a square matrix.
     */
    @Override
    public MatrixProcessorInterface invert() {
        return null;
    }


    /**
     * Inverts a square positive definite matrix.
     */
    @Override
    public MatrixProcessorInterface invertSymmPosDef() {
        return null;
    }


    /**
     * <p>
     * Matrix addition :<br>
     * <br>
     * C = A + B
     * </p>
     */
    @Override
    public MatrixProcessorInterface add() {
        return new Add();
    }


    public static class Add implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix matB = inputs[1].getOriginal();
            Matrix result = null;

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                result = matA.add(matB);
            }

            long elapsedTime = System.nanoTime()-prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(result);
            }
            return elapsedTime;
        }
    }


    /**
     * <p>
     * Matrix multiplication :<br>
     * <br>
     * C = A*B
     * </p>
     */
    @Override
    public MatrixProcessorInterface mult() {
        return new Mult();
    }


    public static class Mult implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix matB = inputs[1].getOriginal();
            Matrix result = null;

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                result = matA.mult(matB);
            }

            long elapsedTime = System.nanoTime()-prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(result);
            }
            return elapsedTime;
        }
    }


    /**
     * <p>
     * Matrix multiplication where B is transposed:<br>
     * <br>
     * C = A*B^T
     * </p>
     */
    @Override
    public MatrixProcessorInterface multTransB() {
        return null;
    }


    /**
     * <p>
     * Multiplies each element in the matrix by a constant value.<br>
     * <br>
     * b<sub>i,j</sub> = &gamma;a<sub>i,j</sub>
     * </p>
     */
    @Override
    public MatrixProcessorInterface scale() {
        return null;
    }


    /**
     * Solve a system with square input matrix:<br>
     * <br>
     * A*X = B<br>
     * <br>
     * where A is an m by m matrix.
     */
    @Override
    public MatrixProcessorInterface solveExact() {
        return null;
    }


    /**
     * Solve a system with a "tall" input matrix:<br>
     * <br>
     * A*X = B<br>
     * <br>
     * where A is an m by n matrix and m > n.
     */
    @Override
    public MatrixProcessorInterface solveOver() {
        return null;
    }


    /**
     * Matrix transpose
     */
    @Override
    public MatrixProcessorInterface transpose() {
        return null;
    }


    @Override
    public BenchmarkMatrix convertToLib(RowMajorMatrix input) {
        return null;
    }


    @Override
    public RowMajorMatrix convertToRowMajor(BenchmarkMatrix input) {
        return null;
    }


    /**
     * String which represents the official library version
     */
    @Override
    public String getLibraryVersion() {
        return null;
    }


    /**
     * The hashcode (i.e. Git SHA) of the source used generate this library. If not available then return an empty
     * string.
     */
    @Override
    public String getSourceHash() {
        return null;
    }


    /**
     * If native code is invoked or not
     */
    @Override
    public boolean isNative() {
        return false;
    }
}