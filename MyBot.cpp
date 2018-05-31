#include "hlt/hlt.hpp"
#include "hlt/navigation.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

using namespace std;

int main(){
	const hlt::Metadata metadata = hlt::initialize("IvanTheTerrible");
	const hlt::PlayerId player_id = metadata.player_id;
  const hlt::Map& initial_map = metadata.initial_map;

	// We now have 1 full minute to analyse the initial map.
	std::ostringstream initial_map_intelligence;
	initial_map_intelligence
	<< "width: " << initial_map.map_width
	<< "; height: " << initial_map.map_height
	<< "; players: " << initial_map.ship_map.size()
	<< "; my ships: " << initial_map.ship_map.at(player_id).size()
	<< "; planets: " << initial_map.planets.size();
	hlt::Log::log( initial_map_intelligence.str() );

	std::vector<hlt::Move> moves;

  // Will be true when my ships have targets associated
  bool moving = false;

  // Maps a target for each of my ships
  map<int, hlt::Entity> myshipsid;

  hlt::Entity target[3];

  target[0].location.pos_x = 0;
  target[0].location.pos_y = initial_map.map_height / 2;

  target[1].location.pos_x = 0;
  target[1].location.pos_y = 0;

  target[2].location.pos_x = 0;
  target[2].location.pos_y = initial_map.map_height;

	for (;;) {
		moves.clear();

		const hlt::Map map = hlt::in::get_map();

    // If ships aren't already moving towards their tagets
    if (!moving) {
      int i = 0;
      // Each ship receives a target
      for (const hlt::Ship& ship : map.ships.at(player_id)) {
        if (i < 3) {
          myshipsid.insert(make_pair(ship.entity_id, target[i]));
          i++;
        }
      }
    }

    moving = true;

    // For each ship move it towards its target
    for (const hlt::Ship& ship : map.ships.at(player_id)) {
      hlt::possibly<hlt::Move> move;
      move = hlt::navigation::navigate_ship_to_dock(
             map, ship, myshipsid[ship.entity_id],
             hlt::constants::MAX_SPEED, moves);
      // Make move if possible
      if (move.second) {
          moves.push_back(move.first);
      }
    }

		if (!hlt::out::send_moves(moves)) {
			hlt::Log::log("send_moves failed; exiting");
			break;
		}
	}
}
