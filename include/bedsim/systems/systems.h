#pragma once

#include "bedsim/systems/blocks.h"
#include "bedsim/systems/collision.h"
#include "bedsim/systems/context.h"
#include "bedsim/systems/inputs.h"
#include "bedsim/systems/liquids.h"
#include "bedsim/systems/movement.h"
#include "bedsim/systems/poses.h"
#include "bedsim/systems/riptide.h"

namespace bedsim::systems {
	template <World W>
	class Systems {
	public:
		Systems(const W& world, const Options& options, Scratch& scratch)
			: context(world, options, scratch) { }

		Systems(const Systems&) = delete;
		Systems& operator=(const Systems&) = delete;

		Context<W> context;
		Collision<W> collision{ context };
		Blocks<W> blocks{ context };
		Poses<W> poses{ collision };
		Liquids<W> liquids{ context, collision, blocks, poses };
		Riptide<W> riptide{ context, liquids };
		Inputs<W> inputs{ context, collision, poses };
		Movement<W> movement{ context, collision, blocks, poses, liquids, riptide };
	};
}
