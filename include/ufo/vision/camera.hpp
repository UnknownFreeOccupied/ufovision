/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the
 * Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of
 * Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_VISION_CAMERA_HPP
#define UFO_VISION_CAMERA_HPP

// UFO
#include <ufo/geometry/shape/ray.hpp>
#include <ufo/math/mat4x4.hpp>
#include <ufo/math/transform3.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/vision/image.hpp>

// STL
#include <cassert>
#include <cmath>
#include <numeric>

namespace ufo
{
enum class ProjectionType { PERSPECTIVE, ORTHOGONAL };

/*!
 * @brief Camera
 *
 */
struct Camera {
	// The pose of the camera in the world, same as the transform that takes you from camera
	// to world
	Transform3f    pose;
	std::size_t    rows;
	std::size_t    cols;
	float          vertical_fov;
	float          near_clip;
	float          far_clip;
	float          zoom;  // TODO: Implement
	Vec3f          up{0.0f, 0.0f, 1.0f};
	ProjectionType projection_type = ProjectionType::PERSPECTIVE;

	template <bool RightHanded = true>
	void lookAt(Vec3f center, Vec3f const& target)
	{
		// We should probably inverse here
		pose = static_cast<Transform3f>(ufo::lookAt<float, RightHanded>(center, target, up));
	}

	[[nodiscard]] Image<Ray3> rays() const
	{
		return rays(execution::seq);
	}

	template <class ExecutionPolicy>
	[[nodiscard]] Image<Ray3> rays(ExecutionPolicy&& policy) const
	{
		switch (projection_type) {
			case ProjectionType::PERSPECTIVE:
				return raysPerspective(std::forward<ExecutionPolicy>(policy));
			case ProjectionType::ORTHOGONAL:
				return raysOrthogonal(std::forward<ExecutionPolicy>(policy));
		}

		return Image<Ray3>(0, 0);
	}

	// TODO: Make private?
 public:
	template <class ExecutionPolicy>
	[[nodiscard]] Image<Ray3> raysPerspective(ExecutionPolicy&& policy) const
	{
		Mat4x4f proj     = projectionPerspective();
		Mat4x4f proj_inv = inverse(proj);

		Mat4x4f view(pose);
		Mat4x4f view_inv = inverse(view);

		// std::cout << "Projection\n" << proj << '\n';
		// std::cout << "View\n" << view << '\n';

		Image<Ray3> rays(rows, cols);

		auto fun = [&](std::size_t row) {
			auto r = ((row + 0.5f) / rows) * 2.0f - 1.0f;
			for (std::size_t col{}; col < cols; ++col) {
				auto  c = ((col + 0.5f) / cols) * 2.0f - 1.0f;
				Vec4f p_nds_h(c, r, -1.0f, 1.0f);
				auto  dir_eye            = proj_inv * p_nds_h;
				dir_eye.w                = 0.0f;
				auto dir_world           = normalize(Vec3f(view_inv * dir_eye));
				rays(row, col).direction = dir_world;
				rays(row, col).origin    = Vec3f(view_inv * Vec4f(Vec3f(0), 1));

				// static auto the_id = std::this_thread::get_id();
				// if (std::this_thread::get_id() == the_id && 0 == row && 0 == col) {
				// 	// std::cout << pose.translation << '\n';
				// 	// std::cout << rays(row, col).direction << "\n\n";
				// }

				// rays(row, col).origin = Vec3f(5.0f, 4.0f, 1.5f);
			}
		};

		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			for (std::size_t row{}; row < rows; ++row) {
				fun(row);
			};

			return rays;
		} else if constexpr (execution::is_par_v<ExecutionPolicy> ||
		                     execution::is_par_unseq_v<ExecutionPolicy>) {
			std::vector<std::size_t> indices(rows);
			std::iota(indices.begin(), indices.end(), 0);
			if constexpr (execution::is_par_v<ExecutionPolicy>) {
				std::for_each(UFO_PAR_STL_PAR indices.begin(), indices.end(), fun);
			} else {
				std::for_each(UFO_PAR_STL_PAR_UNSEQ indices.begin(), indices.end(), fun);
			}
			return rays;
		}
#if defined(UFO_PAR_GCD)
		else if constexpr (execution::is_gcd_v<ExecutionPolicy> ||
		                   execution::is_gcd_unseq_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
#endif
		else if constexpr (execution::is_tbb_v<ExecutionPolicy> ||
		                   execution::is_tbb_unseq_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		} else if constexpr (execution::is_omp_v<ExecutionPolicy> ||
		                     execution::is_omp_unseq_v<ExecutionPolicy>) {
#pragma omp parallel for
			for (std::size_t row = 0; row < rows; ++row) {
				fun(row);
			};
			return rays;
		} else {
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
	}

	template <class ExecutionPolicy>
	[[nodiscard]] Image<Ray3> raysOrthogonal(ExecutionPolicy&& policy) const
	{
		Mat4x4f proj     = projectionOrthogonal();
		Mat4x4f proj_inv = inverse(proj);

		Mat4x4f view(pose);
		Mat4x4f view_inv = inverse(view);

		// std::cout << "Projection\n" << proj << '\n';
		// std::cout << "View\n" << view << '\n';

		Image<Ray3> rays(rows, cols);

		auto fun = [&](std::size_t row) {
			auto r = ((row + 0.5f) / rows) * 2.0f - 1.0f;
			for (std::size_t col{}; col < cols; ++col) {
				auto  c = ((col + 0.5f) / cols) * 2.0f - 1.0f;
				Vec4f p_nds_h(c, r, -1.0f, 1.0f);
				auto  dir_eye              = proj_inv * p_nds_h;
				dir_eye.w                  = 0.0f;
				auto dir_world             = normalize(Vec3f(view_inv * dir_eye));
				rays(row, col).direction   = dir_world;
				rays(row, col).direction.x = 0;
				rays(row, col).direction.y = 0;
				// rays(row, col).direction.z = 0;
				rays(row, col).origin = Vec3f(view_inv * Vec4f(Vec3f(0), 1));

				// static auto the_id = std::this_thread::get_id();
				// if (std::this_thread::get_id() == the_id && 0 == row && 0 == col) {
				// std::cout << pose.translation << '\n';
				// std::cout << rays(row, col).direction << "\n\n";
				// }

				// rays(row, col).origin = Vec3f(5.0f, 4.0f, 1.5f);
			}
		};

		if constexpr (execution::is_seq_v<ExecutionPolicy> ||
		              execution::is_unseq_v<ExecutionPolicy>) {
			for (std::size_t row{}; row < rows; ++row) {
				fun(row);
			};

			return rays;
		} else if constexpr (execution::is_par_v<ExecutionPolicy> ||
		                     execution::is_par_unseq_v<ExecutionPolicy>) {
			std::vector<std::size_t> indices(rows);
			std::iota(indices.begin(), indices.end(), 0);
			if constexpr (execution::is_par_v<ExecutionPolicy>) {
				std::for_each(UFO_PAR_STL_PAR indices.begin(), indices.end(), fun);
			} else {
				std::for_each(UFO_PAR_STL_PAR_UNSEQ indices.begin(), indices.end(), fun);
			}
			return rays;
		}
#if defined(UFO_PAR_GCD)
		else if constexpr (execution::is_gcd_v<ExecutionPolicy> ||
		                   execution::is_gcd_unseq_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
#endif
		else if constexpr (execution::is_tbb_v<ExecutionPolicy> ||
		                   execution::is_tbb_unseq_v<ExecutionPolicy>) {
			// TODO: Implement
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		} else if constexpr (execution::is_omp_v<ExecutionPolicy> ||
		                     execution::is_omp_unseq_v<ExecutionPolicy>) {
#pragma omp parallel for
			for (std::size_t row = 0; row < rows; ++row) {
				fun(row);
			};
			return rays;
		} else {
			static_assert(dependent_false_v<ExecutionPolicy>,
			              "Not implemented for the execution policy");
		}
	}

	[[nodiscard]] Mat4x4f projectionPerspective() const
	{
		if (std::isinf(far_clip)) {
			return infinitePerspective(vertical_fov, cols / static_cast<float>(rows),
			                           near_clip);
		} else {
			return perspective<float, true, true>(vertical_fov, cols / static_cast<float>(rows),
			                                      near_clip, far_clip);
		}
	}

	[[nodiscard]] Mat4x4f projectionOrthogonal() const
	{
		if (std::isinf(far_clip)) {
			return orthogonal<float>(
			    -(static_cast<float>(cols) / 2.0f), static_cast<float>(cols) / 2.0f,
			    static_cast<float>(rows) / 2.0f, -(static_cast<float>(rows) / 2.0f));
		} else {
			return orthogonal<float, true, true>(
			    -(static_cast<float>(cols) / 2.0f), static_cast<float>(cols) / 2.0f,
			    static_cast<float>(rows) / 2.0f, -(static_cast<float>(rows) / 2.0f), near_clip,
			    far_clip);
		}
	}
};

// inline std::ostream &operator<<(std::ostream &out, ufo::Color color)
// {
// 	return out << "Red: " << +color.red << " Green: " << +color.green
// 	           << " Blue: " << +color.blue << " Alpha: " << +color.alpha;
// }
}  // namespace ufo

#endif  // UFO_VISION_CAMERA_HPP