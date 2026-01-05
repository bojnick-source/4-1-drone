/*
===============================================================================
Compliance rules verification and evidence wiring (C++)
File: cpp/engine/compliance/rules_verification.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace lift::compliance {

struct Clause final {
    std::string clause_id;
    std::string title;
    std::string source;
    bool mandatory = true;
    std::vector<std::string> required_evidence_keys;

    void validate() const {
        LIFT_BEMT_REQUIRE(!clause_id.empty(), lift::bemt::ErrorCode::InvalidConfig, "Clause.clause_id empty");
        for (const auto& k : required_evidence_keys) {
            LIFT_BEMT_REQUIRE(!k.empty(), lift::bemt::ErrorCode::InvalidConfig, "Clause.required_evidence_keys contains empty key");
        }
    }
};

struct EvidenceItem final {
    std::string key;
    double value = 0.0;
    std::string unit;
    std::string source;

    void validate() const {
        LIFT_BEMT_REQUIRE(!key.empty(), lift::bemt::ErrorCode::InvalidInput, "EvidenceItem.key empty");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(value), lift::bemt::ErrorCode::InvalidInput, "EvidenceItem.value invalid");
    }
};

struct ComplianceCheck final {
    std::string clause_id;
    bool pass = true;
    std::string message;
};

struct ComplianceReport final {
    lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
    std::vector<ComplianceCheck> checks;

    bool ok() const noexcept {
        if (code != lift::bemt::ErrorCode::Ok) return false;
        for (const auto& c : checks)
            if (!c.pass) return false;
        return true;
    }
};

inline ComplianceReport evaluate_compliance(const std::vector<Clause>& clauses,
                                            const std::vector<EvidenceItem>& evidence) {
    ComplianceReport out;

    std::unordered_map<std::string, const EvidenceItem*> ev_map;
    for (const auto& e : evidence) {
        e.validate();
        ev_map[e.key] = &e; // last wins
    }

    for (const auto& c : clauses) {
        c.validate();
        ComplianceCheck chk;
        chk.clause_id = c.clause_id;
        chk.pass = true;

        for (const auto& key : c.required_evidence_keys) {
            const auto it = ev_map.find(key);
            if (it == ev_map.end() || !it->second || !lift::bemt::is_finite(it->second->value)) {
                chk.pass = false;
                chk.message = "missing or invalid evidence: " + key;
                break;
            }
        }

        if (!c.mandatory && !chk.pass) {
            chk.message = "advisory clause missing evidence: " + chk.message;
        }

        out.checks.push_back(std::move(chk));
    }

    return out;
}

} // namespace lift::compliance

