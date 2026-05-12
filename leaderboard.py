import json
from datetime import datetime
from pathlib import Path


LEADERBOARD_PATH = Path(__file__).resolve().with_name("leaderboard.json")
MAX_LEADERBOARD_ENTRIES = 20


def _team_key(team_number: str):
    return team_number.strip().casefold()


def load_leaderboard():
    if not LEADERBOARD_PATH.exists():
        return []

    try:
        data = json.loads(LEADERBOARD_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []

    if not isinstance(data, list):
        return []

    entries = []
    for item in data:
        if not isinstance(item, dict):
            continue
        team_number = str(item.get("team_number", "")).strip()
        team_name = str(item.get("team_name", "")).strip()
        timestamp = str(item.get("timestamp", "")).strip()
        try:
            score = int(item.get("score", 0))
        except (TypeError, ValueError):
            score = 0

        if not team_number and not team_name:
            continue

        entries.append(
            {
                "team_number": team_number,
                "team_name": team_name,
                "score": score,
                "timestamp": timestamp,
            }
        )

    return _sort_entries(_dedupe_entries(entries))


def add_leaderboard_entry(team_number: str, team_name: str, score: int):
    entries = load_leaderboard()
    new_entry = {
        "team_number": team_number.strip(),
        "team_name": team_name.strip(),
        "score": int(score),
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M"),
    }

    key = _team_key(new_entry["team_number"])
    replaced = False
    for idx, entry in enumerate(entries):
        if _team_key(entry.get("team_number", "")) == key:
            if int(score) > int(entry.get("score", 0)):
                entries[idx] = new_entry
            replaced = True
            break

    if not replaced:
        entries.append(new_entry)

    entries = _sort_entries(_dedupe_entries(entries))[:MAX_LEADERBOARD_ENTRIES]

    try:
        LEADERBOARD_PATH.write_text(json.dumps(entries, indent=2), encoding="utf-8")
    except OSError:
        pass

    return entries


def _sort_entries(entries):
    return sorted(
        entries,
        key=lambda item: (-int(item.get("score", 0)), str(item.get("timestamp", "")), str(item.get("team_number", ""))),
    )


def _dedupe_entries(entries):
    deduped = {}
    for entry in entries:
        key = _team_key(entry.get("team_number", ""))
        if key == "":
            continue

        existing = deduped.get(key)
        if (
            existing is None
            or int(entry.get("score", 0)) > int(existing.get("score", 0))
            or (
                int(entry.get("score", 0)) == int(existing.get("score", 0))
                and str(entry.get("timestamp", "")) >= str(existing.get("timestamp", ""))
            )
        ):
            deduped[key] = {
                "team_number": str(entry.get("team_number", "")).strip(),
                "team_name": str(entry.get("team_name", "")).strip(),
                "score": int(entry.get("score", 0)),
                "timestamp": str(entry.get("timestamp", "")).strip(),
            }

    return list(deduped.values())
