#include "Halfchain.hh"
#include "Edgechain.hh"
#include "Patch.hh"
#include "Circulator.hh"

curvenetwork::Edgechain::Edgechain(int id_)
    : id        (id_)
    , is_deleted()
    , flag      ()
    , num_subdiv()
{}

bool curvenetwork::Edgechain::is_boundary() const {
    return halfchain[0]->is_boundary() || halfchain[1]->is_boundary();
}
