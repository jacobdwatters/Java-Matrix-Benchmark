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
import jmbench.interfaces.BenchmarkMatrix;

/**
 * @author Jacob Watters
 */
public class Flag4jBenchmarkMatrix implements BenchmarkMatrix {

    Matrix mat;

    public Flag4jBenchmarkMatrix(Matrix mat) {
        this.mat = mat;
    }

    public Flag4jBenchmarkMatrix(SparseMatrix mat) {
        this.mat = mat.toDense();
    }


    @Override
    public double get(int row, int col) {
        return mat.get(row, col);
    }


    @Override
    public void set(int row, int col, double value) {
        mat.set(value, row, col);
    }


    @Override
    public int numRows() {
        return mat.numRows;
    }


    @Override
    public int numCols() {
        return mat.numCols;
    }


    @SuppressWarnings("unchecked")
    @Override
    public <T> T getOriginal() {
        return (T) mat;
    }
}
