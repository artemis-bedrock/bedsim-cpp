#pragma once

#include "bedsim/body.h"
#include "bedsim/constants.h"
#include "bedsim/physics.h"
#include "bedsim/systems/context.h"
#include "bedsim/systems/liquids.h"

namespace bedsim::systems {
	struct Launch {
		bool launched{};
		bool known{ true };
	};

	template <World W>
	class Riptide {
	public:
		Riptide(const Context<W>& context, const Liquids<W>& liquids)
			: mContext(context), mLiquids(liquids) { }

		[[nodiscard]] Launch attempt(Body& body, const bool touchingWater) const {
			if (!mContext.hasEquipment() || body.inVehicle || body.riptideTicks > 0 || !body.riptideReady || (!touchingWater && !body.riptideInRain)) {
				return {};
			}

			const int level = mContext.enchantmentLevel(Enchantment::Riptide);
			if (level <= 0 || !body.startingSpinAttack) {
				return {};
			}

			bool headInWater = false;
			if (body.onGround && body.hasGravity && touchingWater) {
				const auto head = mLiquids.observeHead(body);
				if (!head.known) {
					return { .launched = false, .known = false };
				}

				headInWater = head.water;
			}

			body.setVelocity(body.velocity + riptideImpulse(body, level, touchingWater, headInWater));
			body.riptideTicks = kRiptideTicks;
			body.riptideCollision = false;
			body.startingSpinAttack = false;
			return { .launched = true };
		}

	private:
		const Context<W>& mContext;
		const Liquids<W>& mLiquids;
	};
}
