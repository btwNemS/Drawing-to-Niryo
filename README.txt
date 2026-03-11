DRAWING APP - NIRYO NED2
Dessin interactif avec lissage spline → exécution robot

Robot utilisé : Niryo Ned2
Communication via pyniryo
Interface graphique : Tkinter

DESCRIPTION
Application Tkinter permettant de dessiner à la souris.
Le tracé est :
- sous-échantillonné
- lissé par spline (scipy)
- converti en coordonnées robot
- exécuté en trajectoire sur le Niryo Ned2
- le robot dessine sur une surface centrée par rapport à une pose de référence définie dans le code

PREREQUIS

Python 3.9+
Niryo Ned2 accessible sur le réseau
IP robot correcte dans le code :
ROBOT_IP = "xxx.xxx.xxx.xxx"

INSTALLATION
Cloner le projet
git clone <url_du_projet>
cd <nom_du_projet>
Créer un environnement virtuel
python -m venv venv
Activer l'environnement

Windows :
venv\Scripts\activate

Mac / Linux :
source venv/bin/activate

Installer les dépendances
pip install -r requirements.txt

requirements.txt (contenu)
pyniryo
numpy
scipy

Tkinter est inclus avec Python standard.

PARAMETRES IMPORTANTS (dans le code)

Zone réelle de dessin :
DRAW_WIDTH_M = 0.35
DRAW_HEIGHT_M = DRAW_WIDTH_M * 0.5

Profondeur et levée stylo :
Z_DRAW_OFFSET = 0.00
Z_LIFT_OFFSET = 0.03
Z_SAFE_MARGIN = 0.001

Si le stylo n'appuie pas assez :
ajuster Z_DRAW_OFFSET progressivement

Vitesse bras :
ARM_VELOCITY = 100

Qualité du lissage :
MIN_PIXEL_STEP = 2.0
SMOOTHING_FACTOR = 1000.0
INTERPOLATION_POINTS = 150

ORIENTATION DE DESSIN

La pose de référence est définie par :
DRAW_ORIENTATION_JOINTS = JointsPosition(...)

Cette position détermine :
- l'orientation de l'outil
- le centre de la zone de dessin
- la hauteur de base

UTILISATION

Lancer l'application :
python main.py

Souris :
- clic gauche + glisser → dessiner

Clavier :
- Entrée → envoyer au robot
- Espace → annuler le dernier trait
- Backspace → reset complet

PROCESSUS LORS DE L'ENVOI

connexion au robot

calibration automatique

suppression collision détectée

calcul pose de base via forward kinematics

conversion pixel → coordonnées robot

exécution trajectoire par blocs (CHUNK_SIZE)

levée stylo

mouvement final (SALUTATIONS)

fermeture connexion

SECURITE

tester d'abord avec Z_LIFT_OFFSET élevé

vérifier qu'aucun obstacle ne gêne le robot

ajuster la profondeur progressivement

vérifier la bonne calibration avant envoi

ORDRE RAPIDE

python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python main.py