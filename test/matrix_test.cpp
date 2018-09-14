#include "common.h"
#include <vector>

#include <random>
std::default_random_engine generator;
std::uniform_real_distribution<float> distribution(-100.f, 100.f);
std::function<float(void)> randf = std::bind(distribution, generator);

// test fixture
class MatrixTest : public ::testing::Test
{
protected:

  void SetUp() override
  {
    matrices_.resize(NUM_ENTRIES);
    matrices_copy_.resize(NUM_ENTRIES);
    eig_matrices_.resize(NUM_ENTRIES);
    vectors_.resize(NUM_ENTRIES);
    eig_vectors_.resize(NUM_ENTRIES);
    scalars_.resize(NUM_ENTRIES);

    matrices_.clear();
    matrices_copy_.clear();
    eig_matrices_.clear();
    vectors_.clear();
    eig_vectors_.clear();
    scalars_.clear();

    for (size_t i = 0; i < NUM_ENTRIES; i++)
    {
      // random matrices
      eig_matrices_.push_back(100.0f * Eigen::Matrix3f::Random());

      turbomath::Matrix m;
      for (unsigned row = 0; row < 3; row++)
      {
        for (unsigned col = 0; col < 3; col++)
        {
          m(row,col) = eig_matrices_[i](row,col);
        }
      }
      matrices_.push_back(m);
      matrices_copy_.push_back(m);

      // random vectors
      eig_vectors_.push_back(100.0f * Eigen::Vector3f::Random());
      vectors_.push_back(turbomath::Vector(eig_vectors_[i](0), eig_vectors_[i](1), eig_vectors_[i](2)));

      // random scalars
      scalars_.push_back(randf());
    }
  }

  static constexpr size_t NUM_ENTRIES = 1000;

  std::vector<turbomath::Matrix> matrices_;
  std::vector<turbomath::Matrix> matrices_copy_;
  std::vector<Eigen::Matrix3f> eig_matrices_;

  std::vector<turbomath::Vector> vectors_;
  std::vector<Eigen::Vector3f> eig_vectors_;

  std::vector<float> scalars_;
};

constexpr size_t MatrixTest::NUM_ENTRIES;


// convenience functions

#define EXPECT_MATRIX_EQ(m1, m2) { \
  for (unsigned row = 0; row < 3; row++) { \
    for (unsigned col = 0; col < 3; col++) { \
      EXPECT_EQ((m1)(row, col), (m2)(row, col)); \
    } \
  } \
}

#define EXPECT_MATRIX_EIG_EQ(eig, m) { \
  for (unsigned row = 0; row < 3; row++) { \
    for (unsigned col = 0; col < 3; col++) { \
      EXPECT_EQ((eig)(row, col), (m)(row, col)); \
    } \
  } \
}

#define EXPECT_MATRIX_EIG_FLOAT_EQ(eig, m) { \
  for (unsigned row = 0; row < 3; row++) { \
    for (unsigned col = 0; col < 3; col++) { \
      EXPECT_FLOAT_EQ((eig)(row, col), (m)(row, col)); \
    } \
  } \
}

#define EXPECT_VECTOR_EIG_EQ(eig, v) { \
  EXPECT_EQ((eig)(0), (v).x); \
  EXPECT_EQ((eig)(1), (v).y); \
  EXPECT_EQ((eig)(2), (v).z); \
}

#define EXPECT_VECTOR_EIG_FLOAT_EQ(eig, v) { \
  EXPECT_FLOAT_EQ((eig)(0), (v).x); \
  EXPECT_FLOAT_EQ((eig)(1), (v).y); \
  EXPECT_FLOAT_EQ((eig)(2), (v).z); \
}

// void TestEqual(const turbomath::Matrix& m1, const turbomath::Matrix& m2)
// {
//   for (unsigned row = 0; row < 3; row++)
//   {
//     for (unsigned col = 0; col < 3; col++)
//     {
//       EXPECT_EQ(m1(row, col), m2(row, col));
//     }
//   }
// }

// void TestEqual(const turbomath::Matrix& m, const Eigen::Matrix3f& eig)
// {
//   for (unsigned row = 0; row < 3; row++)
//   {
//     for (unsigned col = 0; col < 3; col++)
//     {
//       EXPECT_EQ(m(row, col), eig(row, col));
//     }
//   }
// }

// void TestEqual(const turbomath::Vector& v, const Eigen::Vector3f& eig)
// {
//   EXPECT_EQ(v.x, eig(0));
//   EXPECT_EQ(v.y, eig(1));
//   EXPECT_EQ(v.z, eig(2));
// }

// test cases

TEST_F(MatrixTest, FixtureCheck)
{
  ASSERT_EQ(this->NUM_ENTRIES, matrices_.size());

  for (size_t i = 0; i < this->NUM_ENTRIES; i++)
  {
    bool all_zero = true;
    for (unsigned row = 0; row < 3; row++)
    {
      for (unsigned col = 0; col < 3; col++)
      {
        all_zero = all_zero && (matrices_[i](row, col) == 0.0f);
      }
    }
    ASSERT_FALSE(all_zero);
  }
}

TEST_F(MatrixTest, Access)
{
  turbomath::Matrix m;
  for (unsigned row = 0; row < 3; row++)
  {
    for (unsigned col = 0; col < 3; col++)
    {
      float value = randf();
      m(row, col) = value;
      EXPECT_EQ(value, m(row, col));
    }
  }
}


TEST_F(MatrixTest, Zeros)
{
  turbomath::Matrix m = turbomath::Matrix::zeros();
  for (unsigned row = 0; row < 3; row++)
  {
    for (unsigned col = 0; col < 3; col++)
    {
      EXPECT_EQ(0.0f, m(row, col));
    }
  }
}


TEST_F(MatrixTest, Identity)
{
  turbomath::Matrix m = turbomath::Matrix::identity();
  for (unsigned row = 0; row < 3; row++)
  {
    for (unsigned col = 0; col < 3; col++)
    {
      EXPECT_EQ(row == col ? 1.0f : 0.0f, m(row, col));
    }
  }
}


TEST_F(MatrixTest, CopyConstructor)
{
  for (size_t i = 0; i < NUM_ENTRIES; i++)
  {
    turbomath::Matrix m(matrices_[i]);
    EXPECT_MATRIX_EQ(matrices_[i], m);
  }
}


TEST_F(MatrixTest, Assignment)
{
  for (size_t i = 0; i < NUM_ENTRIES; i++)
  {
    turbomath::Matrix m;
    m = matrices_[i];
    EXPECT_MATRIX_EQ(matrices_[i], m);
  }
}

TEST_F(MatrixTest, Scalar)
{
  for (size_t i = 0; i < NUM_ENTRIES; i++)
  {
    EXPECT_MATRIX_EIG_EQ(scalars_[i]*eig_matrices_[i], scalars_[i]*matrices_[i]);
    EXPECT_MATRIX_EIG_EQ(matrices_copy_[i], matrices_[i]);

    EXPECT_MATRIX_EIG_EQ(eig_matrices_[i]*scalars_[i], matrices_[i]*scalars_[i]);
    EXPECT_MATRIX_EIG_EQ(matrices_copy_[i], matrices_[i]);

    if (scalars_[i] > 0.001f || scalars_[i] < -0.001f)
    {
      EXPECT_MATRIX_EIG_EQ(eig_matrices_[i]/scalars_[i], matrices_[i]/scalars_[i]);
      EXPECT_MATRIX_EIG_EQ(matrices_copy_[i], matrices_[i]);
    }
  }
}

TEST_F(MatrixTest, ScalarMultiplyEqual)
{
  for (size_t i = 0; i < NUM_ENTRIES; i++)
  {
    matrices_[i] *= scalars_[i];
    eig_matrices_[i] *= scalars_[i];
    EXPECT_MATRIX_EQ(eig_matrices_[i], matrices_[i]);
  }
}

TEST_F(MatrixTest, ScalarDivideEqual)
{
  for (size_t i = 0; i < NUM_ENTRIES; i++)
  {
    if (scalars_[i] > 0.001f || scalars_[i] < -0.001f)
    {
      matrices_[i] /= scalars_[i];
      eig_matrices_[i] /= scalars_[i];
      EXPECT_MATRIX_EQ(eig_matrices_[i], matrices_[i]);
    }
  }
}

TEST_F(MatrixTest, MatrixAddition)
{
  for (size_t i = 0; i < NUM_ENTRIES-1; i++)
  {
    EXPECT_MATRIX_EQ(eig_matrices_[i]+eig_matrices_[i+1], matrices_[i]+matrices_[i+1])
    EXPECT_MATRIX_EQ(matrices_copy_[i], matrices_[i]);

    EXPECT_MATRIX_EQ(eig_matrices_[i]-eig_matrices_[i+1], matrices_[i]-matrices_[i+1])
    EXPECT_MATRIX_EQ(matrices_copy_[i], matrices_[i]);
  }
}

TEST_F(MatrixTest, MatrixPlusEqual)
{
  for (size_t i = 0; i < NUM_ENTRIES/2; i++)
  {
    matrices_[i] += matrices_[i+NUM_ENTRIES/2];
    eig_matrices_[i] += eig_matrices_[i+NUM_ENTRIES/2];
    EXPECT_MATRIX_EIG_EQ(eig_matrices_[i], matrices_[i]);
  }
}

TEST_F(MatrixTest, MatrixMinusEqual)
{
  for (size_t i = 0; i < NUM_ENTRIES/2; i++)
  {
    matrices_[i] -= matrices_[i+NUM_ENTRIES/2];
    eig_matrices_[i] -= eig_matrices_[i+NUM_ENTRIES/2];
    EXPECT_MATRIX_EIG_EQ(eig_matrices_[i], matrices_[i]);
  }
}

TEST_F(MatrixTest, MatrixVectorMultiply)
{
  for (size_t i = 0; i < NUM_ENTRIES; i++)
  {
    turbomath::Vector v_result = matrices_[i]*vectors_[i];
    Eigen::Vector3f eig_result = eig_matrices_[i]*eig_vectors_[i];
    EXPECT_VECTOR_EIG_FLOAT_EQ(eig_result, v_result);
    EXPECT_MATRIX_EQ(matrices_copy_[i], matrices_[i]);
  }
}

TEST_F(MatrixTest, MatrixMatrixMultiply)
{
  for (size_t i = 0; i < NUM_ENTRIES-1; i++)
  {
    turbomath::Matrix m_result = matrices_[i]*matrices_[i+1];
    Eigen::Matrix3f eig_result = eig_matrices_[i]*eig_matrices_[i+1];
    EXPECT_MATRIX_EIG_FLOAT_EQ(eig_result, m_result);
    EXPECT_MATRIX_EQ(matrices_copy_[i], matrices_[i]);
  }
}
