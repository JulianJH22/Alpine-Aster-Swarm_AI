# Upload to GitHub — Alpine Aster (Leukerbad Hackathon)

## 1. Create an empty repository

On GitHub, choose **New repository**. Suggested name: `IA-Enjambre`. Choose **Public** if the team wants a public hackathon submission. Set **Add README: Off**, **.gitignore: None**, and **License: None** (select a license later after reviewing third-party rights). Click **Create repository**.

## 2. Upload from Ubuntu / WSL

Unzip the provided archive, then open a terminal in the folder containing `README.md`, `simulacion/`, `ia/`, and `models/`.

```bash
git init
git branch -M main
git add .
git status
git commit -m "Alpine Aster: Leukerbad Hackathon final demo and AI results"
git remote add origin https://github.com/YOUR_USERNAME/IA-Enjambre.git
git push -u origin main
```

Replace `YOUR_USERNAME` with your actual GitHub username. GitHub may ask you to authenticate in a browser or using a personal access token; account passwords are not accepted for HTTPS Git pushes.

If Git requests your identity, run `git config --global user.name "YOUR NAME"` and `git config --global user.email "YOUR_EMAIL"`, then repeat the commit.

## 3. Add demo media

Add the recorded demo video through GitHub Releases or upload it to YouTube, then replace the placeholder in `media/README.md`. Add a real screenshot as `media/captura.png`. Commit and push those changes.

## 4. Final review

Check that `README.md`, `simulacion/`, `ia/fase6/`, `models/`, and `docs/` are visible on GitHub. Review licenses and attribution. Tag a release (`v0.1.0-hackathon`) after verifying the uploaded repository.
