"""
gen_auth.py — Génère le fichier d'authentification EsiBot
==========================================================
Exécuter UNE SEULE FOIS sur le Raspberry Pi pour initialiser les credentials.
Le mot de passe n'est jamais écrit nulle part — seulement son empreinte scrypt.

Usage :
  python3 gen_auth.py
  # Saisir le nom d'utilisateur et le mot de passe à l'invite

Le fichier généré : /home/esibot/.esibot_auth  (chmod 600)
"""

import getpass
import hashlib
import os
import sys

OUTPUT_FILE = os.path.expanduser('/home/esibot/.esibot_auth')


def hash_password(password: str) -> str:
    salt = os.urandom(16)
    key = hashlib.scrypt(password.encode(), salt=salt, n=2**14, r=8, p=1)
    return salt.hex() + ':' + key.hex()


def main():
    print('=== Générateur de credentials EsiBot ===')
    print(f'Fichier de sortie : {OUTPUT_FILE}\n')

    username = input('Nom d\'utilisateur : ').strip()
    if not username:
        sys.exit('Erreur : nom d\'utilisateur vide.')

    password = getpass.getpass('Mot de passe : ')
    if not password:
        sys.exit('Erreur : mot de passe vide.')

    confirm = getpass.getpass('Confirmer le mot de passe : ')
    if password != confirm:
        sys.exit('Erreur : les mots de passe ne correspondent pas.')

    stored = hash_password(password)
    # Effacer les variables sensibles immédiatement
    del password, confirm

    # Écrire le fichier
    with open(OUTPUT_FILE, 'w') as f:
        f.write(f'ESIBOT_USER={username}\n')
        f.write(f'ESIBOT_SCRYPT={stored}\n')

    os.chmod(OUTPUT_FILE, 0o600)
    print(f'\nFichier créé avec succès : {OUTPUT_FILE}  (chmod 600)')
    print('Le mot de passe en clair n\'a été ni écrit ni loggé.')


if __name__ == '__main__':
    main()
