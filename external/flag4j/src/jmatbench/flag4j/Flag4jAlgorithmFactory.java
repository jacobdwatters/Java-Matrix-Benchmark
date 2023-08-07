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
import com.flag4j.SparseMatrix;
import com.flag4j.linalg.decompositions.RealCholeskyDecomposition;
import com.flag4j.linalg.decompositions.RealLUDecomposition;
import com.flag4j.linalg.decompositions.RealQRDecomposition;
import com.flag4j.linalg.decompositions.RealSVD;
import com.flag4j.linalg.solvers.RealExactSolver;
import com.flag4j.linalg.solvers.RealLstsqSolver;
import jmbench.interfaces.BenchmarkMatrix;
import jmbench.interfaces.MatrixProcessorInterface;
import jmbench.interfaces.RuntimePerformanceFactory;
import jmbench.matrix.RowMajorMatrix;
import jmbench.tools.BenchmarkConstants;

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
        return new Chol();
    }


    public static class Chol implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix L = null;

            RealCholeskyDecomposition chol = new RealCholeskyDecomposition();

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                chol.decompose(matA);
                L = chol.getL();
            }

            long elapsedTime = System.nanoTime() - prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(L);
            }
            return elapsedTime;
        }
    }

    /**
     * LU decomposition
     */
    @Override
    public MatrixProcessorInterface lu() {
        return new LU();
    }


    public static class LU implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();

            Matrix L=null, U=null;
            SparseMatrix P=null;
            RealLUDecomposition lu = new RealLUDecomposition();

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                lu.decompose(matA);
                L=lu.getL();
                U=lu.getU();
                P=lu.getP();
            }

            long elapsedTime = System.nanoTime() - prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(L);
                outputs[1] = new Flag4jBenchmarkMatrix(U);
                outputs[2] = new Flag4jBenchmarkMatrix(P);
            }
            return elapsedTime;
        }
    }


    /**
     * Singular Value Decomposition
     */
    @Override
    public MatrixProcessorInterface svd() {
        // TODO: SVD is really really slow. Do not use.
        return null;
    }


    public static class SVD implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();

            Matrix U=null,W=null,V=null;
            RealSVD s = new RealSVD(true, true);

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                s.decompose(matA);
                U=s.getU();
                W=s.getS();
                V=s.getV();
            }

            long elapsedTime = System.nanoTime() - prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(U);
                outputs[1] = new Flag4jBenchmarkMatrix(W);
                outputs[2] = new Flag4jBenchmarkMatrix(V);
            }
            return elapsedTime;
        }
    }


    /**
     * QR Decomposition
     */
    @Override
    public MatrixProcessorInterface qr() {
        return new QR();
    }


    public static class QR implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();

            Matrix Q=null,R=null;
            RealQRDecomposition qr = new RealQRDecomposition();

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                qr.decompose(matA);
                Q=qr.getQ();
                R=qr.getR();
            }

            long elapsedTime = System.nanoTime() - prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(Q);
                outputs[1] = new Flag4jBenchmarkMatrix(R);
            }
            return elapsedTime;
        }
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
        return new Det();
    }


    public static class Det implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                matA.det();
            }

            return System.nanoTime() - prev;
        }
    }


    /**
     * Inverts a square matrix.
     */
    @Override
    public MatrixProcessorInterface invert() {
        return new Inv();
    }

    public static class Inv implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix result = null;

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                result = matA.inv();
            }

            long elapsedTime = System.nanoTime()-prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(result);
            }
            return elapsedTime;
        }
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
        return new MultTransB();
    }


    public static class MultTransB implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix matB = inputs[1].getOriginal();
            Matrix result = null;


            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                result = matA.multTranspose(matB);
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
     * Multiplies each element in the matrix by a constant value.<br>
     * <br>
     * b<sub>i,j</sub> = &gamma;a<sub>i,j</sub>
     * </p>
     */
    @Override
    public MatrixProcessorInterface scale() {
        return new Scale();
    }


    public static class Scale implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix result = null;

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                result = matA.mult(BenchmarkConstants.SCALE);
            }

            long elapsedTime = System.nanoTime()-prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(result);
            }
            return elapsedTime;
        }
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
        return new SolveExact();
    }


    public static class SolveExact implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix matB = inputs[1].getOriginal();

            Matrix result = null;
            RealExactSolver solver = new RealExactSolver();

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                result = solver.solve(matA, matB);
            }

            long elapsedTime = System.nanoTime()-prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(result);
            }
            return elapsedTime;
        }
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
        return new SolveLSTSQ();
    }


    public static class SolveLSTSQ implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix matB = inputs[1].getOriginal();

            Matrix result = null;
            RealLstsqSolver solver = new RealLstsqSolver();

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                result = solver.solve(matA, matB);
            }

            long elapsedTime = System.nanoTime()-prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(result);
            }
            return elapsedTime;
        }
    }


    /**
     * Matrix transpose
     */
    @Override
    public MatrixProcessorInterface transpose() {
        return new Transpose();
    }

    public static class Transpose implements MatrixProcessorInterface {
        @Override
        public long process(BenchmarkMatrix[] inputs, BenchmarkMatrix[] outputs, long numTrials) {
            Matrix matA = inputs[0].getOriginal();
            Matrix result = null;

            long prev = System.nanoTime();

            for( long i = 0; i < numTrials; i++ ) {
                result = matA.T();
            }

            long elapsedTime = System.nanoTime()-prev;
            if( outputs != null ) {
                outputs[0] = new Flag4jBenchmarkMatrix(result);
            }
            return elapsedTime;
        }
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
        return "v0.0.1-beta";
    }


    /**
     * The hashcode (i.e. Git SHA) of the source used generate this library. If not available then return an empty
     * string.
     */
    @Override
    public String getSourceHash() {
        return "";
    }


    /**
     * If native code is invoked or not
     */
    @Override
    public boolean isNative() {
        return false;
    }
}