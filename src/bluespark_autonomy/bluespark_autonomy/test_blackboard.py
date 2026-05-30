import py_trees

def test_blackboard_mechanics():
    print("\n--- START TESTÓW ARCHITEKTURY BLACKBOARDU ---\n")
    
    # 1. TWARDY RESET
    # Gwarantuje, że tablica jest czysta przed startem misji/testu
    py_trees.blackboard.Blackboard.clear()
    
    # 2. SYMULACJA ZARZĄDCY (BlackboardManager)
    # Rejestrujemy klienta z uprawnieniami do zapisu
    print("Inicjalizacja głównego zarządcy (MasterManager)...")
    writer_client = py_trees.blackboard.Client(name="MasterManager")
    writer_client.register_key(key="vision/detected_objects", access=py_trees.common.Access.WRITE)
    
    # Zarządca odbiera "fejkowe" dane z kamery i zapisuje je na tablicy
    mock_vision_data = {"Gate": {"x": 100, "y": 200, "distance": 1.5}}
    writer_client.set("vision/detected_objects", mock_vision_data)
    
    assert writer_client.get("vision/detected_objects") == mock_vision_data
    print("[OK] Zarządca pomyślnie zapisał stan świata na tablicy.")

    # 3. SYMULACJA KLOCKA W DRZEWIE (np. IsObjectVisible)
    # Rejestrujemy klienta warunku z uprawnieniami TYLKO do odczytu
    print("\nInicjalizacja klocka decyzyjnego (IsObjectVisible)...")
    reader_client = py_trees.blackboard.Client(name="ConditionNode")
    reader_client.register_key(key="vision/detected_objects", access=py_trees.common.Access.READ)
    
    # Klocek czyta dane
    dane_dla_klocka = reader_client.get("vision/detected_objects")
    assert "Gate" in dane_dla_klocka
    print("[OK] Klocek poprawnie i natychmiastowo odczytał dane z tablicy.")

    # 4. TEST OCHRONY (Race Condition Prevention)
    # Sprawdzamy, co się stanie, gdy ktoś napisze zły kod w klocku drzewa 
    # i spróbuje nadpisać zmienną, do której ma tylko odczyt.
    print("\nTestowanie zabezpieczeń frameworka (próba nielegalnego zapisu)...")
    try:
        reader_client.set("vision/detected_objects", {"ZleDane": "KasujeBramke"})
        assert False, "BŁĄD: System pozwolił na nielegalny zapis!"
    except AttributeError as e:  # <-- O TUTAJ ZMIANA
        print(f"[OK] System zablokował zapis i rzucił błąd. Architektura jest szczelna.\n(Komunikat frameworka: {e})")

    print("\n--- WSZYSTKIE TESTY ZALICZONE ---")
if __name__ == '__main__':
    test_blackboard_mechanics()
