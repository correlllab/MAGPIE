# DeliGrasp
 
 Active development branch and monorepo of sorts for DeliGrasp, Just Add Force, and future/ongoing research projects.

 Use the provided environment.yml to create the Conda environment (contains several large installs of tf, torch, jax, cuda). This soup works for me (running a GTX 2070 on Ubuntu 22.04.5 LTS, driver version 550.120), but no guarantees it does for you. Unsure which package needs which CUDA version, but yes, both 11.8 and 12.4 are used...

 Features available: grasping synthesis frameworks via 1) DeliGrasp: compositional VLM (OWLv2) + segmentation (SAM2) + LLM (grasp reasoning) or 2) Just Add Force: end to end (and far less functional) grasping diffusion policy trained on DeliGrasp trajectories.

 For the most part, feature functionality is exposed to the user via a Flask webapp.

Run the webserver via the following commands.
```
cd webserver
python server.py
```

Webapp instructions forthcoming (soon...)