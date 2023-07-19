const { execSQL } = require("./mysql");
const express = require("express");
const app = express();

app.use("/", express.static("public"));

app.get("/data", async (req, res) => {
  const sql = "SELECT * FROM poi";

  try {
    const result = await execSQL(sql);
    res.json(result);
  } catch (error) {
    console.error(error);
    res.json({ error });
  }
});

app.listen(3000, () => {
  console.log("Server is running at http://localhost:3000");
});

